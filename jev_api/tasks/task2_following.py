"""Task 2: rear-region maintenance/recovery and an independent STOP."""

import math

from action_space import (FOLLOWING_ACTION_CHOICES as ACTION_CHOICES,
                          FOLLOWING_COURSES_DEG as COURSES_DEG,
                          FOLLOWING_DIRECTIONS as DIRECTIONS, SPEED_FRACTIONS)
from geometry.coordinates import KNOT_MPS
from geometry.following import FollowingGeometry, observed_motion, request_phase
from geometry.motion import predict_ground_motion, preview_duration
from geometry.obstacles import nearby_obstacles, clearance_facts, snapshot_clearance
from geometry.relative_motion import course_error


SAFETY_INSTRUCTIONS = (
    'Choose ONE action. First avoid hull contact and preserve NONNEGATIVE safety clearance for ALL hazards. '
    'Brake/turn early using previews; if already unsafe, improve end clearance. '
    'Use nonzero speed to turn. '
    'Only the first interval executes, then replan; long forecasts assume holding this command.'
)
INSTRUCTIONS = {
    'maintain': SAFETY_INSTRUCTIONS + 'MAINTAIN / SMALL CORRECTION: continue tracking the soft rear point '
        'using point errors and the course reference. A safe detour may increase point error.',
    'recover': SAFETY_INSTRUCTIONS + 'LOST / RECOVER: enter the rear interior with margin promptly, then settle. '
        'Target may keep turning OR straighten: prefer progress shared by both forecasts; do not rely on a continued turn. '
        'Compare CW/CCW bypass distances and make sustained progress on a safe side; do not alternate sides without cause. '
        'If too close, separate first. Ahead at matching speed or stopped is not recovered. '
        'A detour can temporarily increase straight region error. Then resume following.',
}
SCHEMA = {
    'motion': 'Commands are target speeds and absolute north-clockwise courses. Zero speed brakes over time and ignores course.',
    'obstacles': 'Obstacle hull/safety clearance arrays=[action min,end,full-preview min,end], metres; <=0 hull contact, <0 safety intrusion. Current-position snapshots, not future obstacle paths.',
}


def validate_params(params):
    lower, upper, maximum = (params.get(k) for k in ('followDistanceMinM', 'followDistanceMaxM', 'maxSpeed'))
    if (type(params.get('targetPlatCode')) is not int or not 0 <= params['targetPlatCode'] <= 65535
            or params['targetPlatCode'] == params['platCode']
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in (lower, upper, maximum))
            or not 0 <= lower <= upper or upper <= 0 or maximum < 0):
        raise ValueError('Invalid followMode parameters')


def target_platform_id(params):
    return params['targetPlatCode']


def validate_manifest(params, radii, expected):
    target = params['targetPlatCode']
    if target not in expected:
        raise ValueError('Missing following target in current snapshot')
    if params['followDistanceMaxM'] <= radii[str(target)] + radii[str(params['platCode'])]:
        raise ValueError('Following region entirely inside combined hulls')


def adapt_observation(params, local, records, obs, state):
    target = next(r for r in records if r['platformId'] == params['targetPlatCode'])
    speed, course = target['targetSpeed'], target['targetCourse']
    if (type(speed) not in (int, float) or not math.isfinite(speed) or not 0 <= speed <= 200
            or (course is None and speed > 1e-5)
            or (course is not None and (type(course) not in (int, float) or not math.isfinite(course) or not 0 <= course < 360))):
        raise ValueError('Unknown or unsupported following target motion')
    stationary = speed*KNOT_MPS < (.2 if state.get('stationary', False) else .1)
    reference = state.get('reference_course') if stationary else course
    if reference is None:
        reference = course if course is not None else local.get('target_reference_course_deg')
    if type(reference) not in (int, float) or not math.isfinite(reference) or not 0 <= reference < 360:
        raise ValueError('Following target needs a reliable initial reference direction')
    motion = observed_motion(state, local, speed, reference, stationary)
    state.update(stationary=stationary, reference_course=reference)
    obs['target'] = dict(target, radius_m=local['collision_radii_m'][str(params['targetPlatCode'])])
    obs['task'] = dict(params, code='followMode', reference_course_deg=reference,
                       target_stationary=stationary, target_motion=motion)
    maximum = min(local['max_speed_knots'], params['maxSpeed']) if params['maxSpeed'] > 0 else local['max_speed_knots']
    obs['action_space']['speeds_knots'] = [maximum*f for f in SPEED_FRACTIONS]
    if obs.get('operation') != 'stop':
        context = request_phase(FollowingGeometry(obs), state.get('following_phase'))
        obs['task']['following_request'] = context
        state['following_phase'] = context['phase']


def validate_observation(obs, finite):
    task, target = obs['task'], obs['target']
    validate_params(task)
    return (all(finite(target[k]) for k in ('longitude', 'latitude', 'targetSpeed', 'radius_m'))
            and -180 <= target['longitude'] <= 180 and -90 <= target['latitude'] <= 90
            and 0 <= target['targetSpeed'] <= 200 and target['radius_m'] > 0
            and (target['targetCourse'] is None and target['targetSpeed'] == 0
                 or finite(target['targetCourse']) and 0 <= target['targetCourse'] < 360)
            and finite(task['reference_course_deg']) and 0 <= task['reference_course_deg'] < 360
            and all(finite(task['target_motion'][k]) for k in ('yaw_deg_s', 'acceleration_mps2'))
            and task['followDistanceMaxM'] > target['radius_m']+obs['boat']['collision_radius_m'])


def forecasts(obs, geometry, times, obstacles):
    """Yield every original choice with unranked physical consequences."""
    execution = obs['execution']
    speed = obs['boat']['targetSpeed']*KNOT_MPS
    for name, (i, j) in ACTION_CHOICES.items():
        command = obs['action_space']['speeds_knots'][i]*KNOT_MPS
        course = obs['action_space']['courses_deg'][j]
        motions, positions = [], []
        for index, t in enumerate(times):
            dt = execution['fixed_dt_s'] if index == 0 else max(.1, execution['fixed_dt_s'])
            motion = predict_ground_motion(speed, command, course, execution, t, dt=dt, route_details=False)
            motions.append(motion)
            positions.append(geometry.forecast(motion, t, dt))
        facts = dict(speed_mps=command, course_deg=course)
        if obstacles:
            clear = clearance_facts(motions[0], motions[-1], obstacles, obs['boat']['collision_radius_m'])
            facts['obstacle_hull_clearance_m'] = clear['obstacle_hull_clearance_m']
            # Include the action endpoint for recovery from a shared unsafe start.
            action_end = snapshot_clearance([motions[0]['endpoint']]*2, obstacles,
                                            obs['boat']['collision_radius_m'], 'avoidance_radius_m')
            facts['obstacle_safety_clearance_m'] = [clear['obstacle_safety_clearance_m'][0], action_end,
                                                   *clear['obstacle_safety_clearance_m'][1:]]
        yield name, facts, positions, motions


def build_payload(obs, model='jev-latest'):
    geometry = FollowingGeometry(obs)
    duration = obs['planning']['action_duration_s']
    times = [duration, 12*duration]
    safety_horizon = max(times[-1], preview_duration(obs['execution'], duration, obs['boat']['targetSpeed']*KNOT_MPS))
    prediction_times = times + ([safety_horizon] if safety_horizon > times[-1] else [])
    context = obs['task'].get('following_request') or request_phase(geometry)
    phase = context['phase']
    obstacles = nearby_obstacles(obs)
    state = dict(task=dict(mode=phase, classification=context,
                           allowed_distance_m=[obs['task']['followDistanceMinM'], obs['task']['followDistanceMaxM']],
                           allowed_relative_bearing_deg=[90,270],
                           minimum_target_distance_m=obs['task']['followDistanceMinM'],
                           objective='Continue soft rear-point tracking.' if phase == 'maintain' else 'Safely regain the rear region promptly.'),
                 current_region={k:geometry.current[k] for k in ('distance_m','relative_bearing_deg','in_sector','sector_error_m')},
                 boat=dict(actual_speed_mps=obs['boat']['targetSpeed']*KNOT_MPS,
                           heading_deg=obs['execution']['heading_course_deg'],
                           yaw_deg_s=obs['execution']['yaw_rate_deg_s']),
                 target=dict(reference_course_deg=geometry.course, actual_speed_mps=geometry.speed,
                             observed_motion=obs['task']['target_motion']),
                 combined_hull_radius_m=geometry.combined_radius,
                 forecast_times_s=times, safety_preview_s=safety_horizon,
                 forecast=('Tracking point translates at current target velocity/direction. ' if phase == 'maintain' else
                           'Recovery region follows the observed target turn. ')
                           +'Safety retains observed target turn and at most 2s acceleration. Own bounded acceleration/yaw. Only first interval executes. Unknown manoeuvres are not predicted.',
                 obstacles=obstacles, candidate_schema=SCHEMA)
    if phase == 'recover':
        state['interior'] = dict(distance_m=[geometry.inner,geometry.outer],
                                 relative_bearing_deg=[90+geometry.angle_margin,270-geometry.angle_margin])
        state['region_error_m'] = geometry.region(geometry.relative)['sector_error_m']
        remaining, references = geometry.bypass(geometry.relative, times[-1])
        state['bypass_remaining_m'] = remaining
        state['bypass_reference_courses_deg'] = references
    else:
        guidance = geometry.guidance(times[-1])
        state['task'].update(point_behind_m=geometry.distance,point_tolerance_m=geometry.tolerance)
        state['tracking_error_m'] = geometry.point_error(geometry.relative)
        state['guidance'] = guidance
    criteria = {}
    for name, facts, positions, motions in forecasts(obs, geometry, prediction_times, obstacles):
        ranges = [positions[0]['minimum_m'], positions[0]['distance_m'],
                  positions[-1]['minimum_m'], positions[-1]['distance_m']]
        required = max(geometry.combined_radius, obs['task']['followDistanceMinM'])
        hull = [r-geometry.combined_radius for r in ranges]
        safety = [r-required for r in ranges]
        if obstacles:
            hull = [min(a,b) for a,b in zip(hull,facts['obstacle_hull_clearance_m'])]
            safety = [min(a,b) for a,b in zip(safety,facts['obstacle_safety_clearance_m'])]
        facts = dict(speed_mps=facts['speed_mps'], course_deg=facts['course_deg'],
                     hull_clearance_m=hull, safety_clearance_m=safety)
        if phase == 'recover':
            facts['region_error_m'] = [p['region_error_m'] for p in positions[:2]]
            facts['coast_region_error_m'] = [p['coast_region_error_m'] for p in positions[:2]]
            facts['bypass_remaining_m'] = [p['bypass_remaining_m'] for p in positions[:2]]
        else:
            facts['tracking_error_m'] = [p['tracking_error_m'] for p in positions[:2]]
            if guidance['course_deg'] is not None:
                facts['guidance_error_deg'] = [abs(course_error(guidance['course_deg'],c)) for c in (facts['course_deg'],motions[0]['end_course_deg'])]
        criteria[name] = facts
    return dict(model=model, state=state,
                questions=dict(action=dict(type='choice', instructions=INSTRUCTIONS[phase], criteria=criteria)))


def build_stop_payload(obs, model='jev-latest'):
    """Separate objective/schema; no following-region or acquisition instructions."""
    geometry = FollowingGeometry(obs)
    duration = obs['planning']['action_duration_s']
    horizon = preview_duration(obs['execution'], duration, obs['boat']['targetSpeed']*KNOT_MPS)
    obstacles = nearby_obstacles(obs)
    criteria = {}
    for name, facts, positions, motions in forecasts(obs, geometry, [duration, horizon], obstacles):
        facts.update(end_speed_mps=motions[0]['end_speed'],
                     travel_m=math.hypot(*motions[0]['endpoint']),
                     target_hull_clearance_m=[p['minimum_m']-obs['boat']['collision_radius_m']-obs['target']['radius_m'] for p in positions])
        criteria[name] = facts
    state = dict(task=dict(mode='immediate_stop', stop_speed_threshold_mps=obs['task']['stop_speed_mps']),
                 boat=dict(actual_speed_mps=obs['boat']['targetSpeed']*KNOT_MPS),
                 forecast_times_s=[duration, horizon], obstacles=obstacles,
                 candidate_schema=dict(encoding='STOP numeric rows use candidate_value_columns order; distances/angles at 0.1, speeds at 0.001. All 128 choices remain available.', motion=SCHEMA['motion'], obstacles=SCHEMA['obstacles'],
                     stop='end_speed is actual speed after first interval; travel_m is endpoint displacement. target_hull_clearance_m=[action min,full-preview min]; <=0 contact. Target uses observed motion; obstacles frozen.'))
    instructions = ('IMMEDIATE STOP: choose ZERO target speed to brake to rest and keep it zero once stopped. '
                    'Course is ignored at zero speed; braking takes time. Following objectives are suspended; '
                    'target separation is not a reason to resume following. Physical collision facts remain visible. '
                    'Only the first interval executes; the longer held-command preview is hypothetical.')
    return dict(model=model, state=state,
                questions=dict(action=dict(type='choice', instructions=instructions, criteria=criteria)))


def status_params(context, observation=None):
    result = dict(targetPlatCode=context.get('targetPlatCode'))
    if observation is not None:
        geometry = FollowingGeometry(observation)
        current = geometry.current
        result.update(relativeDistanceM=current['distance_m'], relativeBearingDeg=current['relative_bearing_deg'],
                      inRange=int(current['in_range']), distanceOutM=current['distance_out_m'],
                      targetSpeed=observation['target']['targetSpeed'], targetCourse=geometry.course)
    else:
        for key in ('relativeDistanceM', 'relativeBearingDeg', 'inRange', 'distanceOutM', 'targetSpeed', 'targetCourse'):
            if key in context:
                result[key] = context[key]
    return result
