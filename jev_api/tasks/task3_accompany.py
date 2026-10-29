"""Task 3: no-obstacle accompaniment; two explicit request variants."""

import math

from action_space import ACTION_CHOICES, SPEED_FRACTIONS
from geometry.accompany import AccompanyGeometry
from geometry.coordinates import KNOT_MPS
from geometry.motion import predict_ground_motion
from geometry.relative_motion import course_error
from tasks.stop import build_waypoint_stop_payload

BOUNDS = ('offsetAlongMinM', 'offsetAlongMaxM', 'offsetAcrossMinM', 'offsetAcrossMaxM')
ACQUIRE_INSTRUCTIONS = (
    'ACQUIRE CONTROL REGION: enter the interior reference rectangle, leaving margin to the customer band. '
    'Choose ONE command which actively closes control_error_m. control_gain_m is current control-region '
    'error minus next-action control-region error: '
    'positive means improvement, zero stagnation, negative worsening. Do not match target velocity and '
    'remain outside the control region. Some relative velocity is REQUIRED to acquire position. Use a short corrective '
    'speed/course pulse; you will reconsider next second. A long held-command overshoot is NOT a reason '
    'to freeze outside: apply a useful action now, then brake in subsequent decisions. Preserve the outer '
    'customer band while moving inward. If turning away, '
    'compare multi-second error to keep a useful turn consistent, allowing initial error growth. '
    'Aim for the band centre for margin. Avoid hull contact (min_clearance_m). Commands have bounded '
    'acceleration and yaw; zero speed cannot rotate at rest. Along + forward, across + starboard. '
)
POSITION_INSTRUCTIONS = (
    'Choose ONE command to continuously HOLD the interior control rectangle, leaving margin to the '
    'outer customer band. control_error_m gives distance outside the interior rectangle at forecast_times_s. '
    'band_error_m gives distance outside the customer band. Correct BOTH axes. Matching '
    'velocity at the wrong position is ineffective. A useful turn may initially increase error. '
    'As error becomes small, brake relative motion early to prevent crossing the opposite boundary. '
    'Use the band centre as a soft reference to retain margin. Inside, reduce body-frame position rates. '
    'Body-frame rate zero holds position, even in a target turn; world relative velocity need not be zero. '
    'Avoid negative min_clearance_m. '
    'Commands are target speeds and absolute north-clockwise courses, with bounded acceleration and yaw. '
    'They execute for one action and will be reconsidered; longer forecast assumes holding the command. '
    'Keep useful turns consistent. Along + forward, across + starboard. Zero speed cannot rotate at rest. '
)
CANDIDATE_SCHEMA = {'position': 'position_m rows are [along,across] at forecast_times_s. '
          'Compare each coordinate against the task intervals; the band centre is a soft reference.',
 'error': 'band_error_m is distance outside customer bounds; control_error_m is distance outside '
          'the middle-half control rectangle with 0.05m observation tolerance. Both follow forecast_times_s.',
 'progress': 'control_gain_m = current control-region error minus first forecast control-region error.',
 'guidance': 'guidance_error_deg = [command course error, first forecast heading error] to guidance.course_deg. '
             'Use for consistent steering, while comparing speed/braking/position consequences.',
 'holding': 'end_rate and preview_rate are [along,across] target-frame position derivatives at the first '
            'and last forecast times. Zero holds position, including in a turn. Suffix indicates mps or knots.',
 'clearance': 'min_clearance_m is minimum swept friendly hull clearance through the full preview; <=0 means contact.',
 'forecast': 'Held command; bounded own acceleration/yaw; observed target constant turn and at most '
             'two seconds of measured acceleration, then constant speed. Unknown manoeuvres are not predicted.'}


def validate_params(params):
    if (type(params.get('targetPlatCode')) is not int or not 0 <= params['targetPlatCode'] <= 65535
            or params['targetPlatCode'] == params['platCode']
            or any(type(params.get(k)) not in (int, float) or not math.isfinite(params[k]) for k in BOUNDS)
            or params['offsetAlongMinM'] > params['offsetAlongMaxM']
            or params['offsetAcrossMinM'] > params['offsetAcrossMaxM']
            or type(params.get('syncHeading')) is not int or params['syncHeading'] not in (0, 1)
            or type(params.get('maxSpeed')) not in (int, float) or not math.isfinite(params['maxSpeed']) or params['maxSpeed'] < 0):
        raise ValueError('Invalid accompanyMode parameters')


def target_platform_id(params):
    return params['targetPlatCode']


def validate_manifest(params, radii, expected):
    if params['targetPlatCode'] not in expected:
        raise ValueError('Missing accompanying target in current snapshot')
    if len(expected) != 2:
        raise ValueError('Accompaniment currently supports own boat and friendly target only; disable obstacles')
    nearest = [min(hi, max(lo, 0)) for lo, hi in (
        (params['offsetAlongMinM'], params['offsetAlongMaxM']),
        (params['offsetAcrossMinM'], params['offsetAcrossMaxM']))]
    if math.hypot(*nearest) <= radii[str(params['platCode'])] + radii[str(params['targetPlatCode'])]:
        raise ValueError('Accompany position band overlaps combined target/own hulls')


def adapt_observation(params, local, records, obs, state):
    target = next(r for r in records if r['platformId'] == params['targetPlatCode'])
    speed, course = target['targetSpeed'], target['targetCourse']
    if speed is None or not 0 <= speed <= 200 or (speed > 0 and course is None):
        raise ValueError('Unknown or unsupported friendly target motion')
    reference = course if course is not None else local['target_reference_course_deg']
    if type(reference) not in (int, float) or not math.isfinite(reference) or not 0 <= reference < 360:
        raise ValueError('Invalid target reference direction')
    obs['target'] = dict(target, radius_m=local['collision_radii_m'][str(params['targetPlatCode'])])
    obs['task'] = dict(params, code='accompanyMode', reference_course_deg=reference,
                       request_variant='heading_sync' if params['syncHeading'] else 'position_only')
    # Only completed snapshots contribute history. Reset/tick rollback never crosses episodes.
    tick = local['tick']
    previous = state.get('accompany_motion')
    yaw = acceleration = 0.0
    if previous and tick > previous['tick']:
        dt = (tick-previous['tick'])*local['execution']['fixed_dt_s']
        if 0 < dt <= 5:
            alpha = dt/(.7+dt)
            yaw = previous['yaw'] + alpha*(course_error(reference, previous['course'])/dt-previous['yaw'])
            acceleration = previous['acceleration'] + alpha*((speed-previous['speed'])*KNOT_MPS/dt-previous['acceleration'])
    state['accompany_motion'] = dict(tick=tick, course=reference, speed=speed, yaw=yaw, acceleration=acceleration)
    obs['task']['target_motion'] = dict(yaw_deg_s=yaw if abs(yaw) >= .05 else 0,
                                       acceleration_mps2=acceleration if abs(acceleration) >= .15 else 0)
    maximum = min(local['max_speed_knots'], params['maxSpeed']) if params['maxSpeed'] > 0 else local['max_speed_knots']
    obs['action_space']['speeds_knots'] = [maximum*f for f in SPEED_FRACTIONS]


def validate_observation(obs, finite):
    task, target = obs['task'], obs['target']
    validate_params(task)
    return (not obs['obstacles'] and all(finite(target[k]) for k in ('longitude', 'latitude', 'targetSpeed', 'radius_m'))
            and -180 <= target['longitude'] <= 180 and -90 <= target['latitude'] <= 90
            and 0 <= target['targetSpeed'] <= 200 and target['radius_m'] > 0
            and (target['targetCourse'] is None and target['targetSpeed'] == 0
                 or finite(target['targetCourse']) and 0 <= target['targetCourse'] < 360)
            and finite(task['reference_course_deg']) and 0 <= task['reference_course_deg'] < 360
            and task['request_variant'] == ('heading_sync' if task['syncHeading'] else 'position_only'))


def build_payload(obs, model='jev-latest'):
    geometry = AccompanyGeometry(obs)
    boat, task, execution = obs['boat'], obs['task'], obs['execution']
    duration = obs['planning']['action_duration_s']
    horizon = max(12, 90/execution['max_yaw_rate_deg_s'] +
                  execution['max_yaw_rate_deg_s']/execution['max_angular_acceleration_deg_s2'],
                  abs(boat['targetSpeed'])*KNOT_MPS/execution['max_deceleration_mps2'] + duration)
    times = [max(3*duration, horizon/2), max(6*duration, horizon)]
    synchronize = task['request_variant'] == 'heading_sync'
    position_rate = math.hypot(geometry.current['along_rate_mps'], geometry.current['across_rate_mps'])
    heading_active = synchronize and geometry.current['in_control_region'] and position_rate <= 1 and abs(geometry.yaw) < .05
    acquiring = not geometry.current['in_control_region']
    centre = geometry.reference()
    guidance = geometry.guidance(times[-1])
    state = dict(
        task=dict(mode='accompany', objective='position_and_heading' if synchronize else 'position_only',
                  stage='acquire_position' if acquiring else 'hold_position',
                  allowed_along_m=[task['offsetAlongMinM'], task['offsetAlongMaxM']],
                  allowed_across_m=[task['offsetAcrossMinM'], task['offsetAcrossMaxM']],
                  control_along_m=[geometry.control_task['offsetAlongMinM'], geometry.control_task['offsetAlongMaxM']],
                  control_across_m=[geometry.control_task['offsetAcrossMinM'], geometry.control_task['offsetAcrossMaxM']]),
        relative=geometry.current, band_centre=centre, guidance=guidance,
        boat=dict(actual_speed_mps=boat['targetSpeed']*KNOT_MPS,
                  hull_radius_m=boat['collision_radius_m']),
        target=dict(actual_speed_mps=geometry.target_speed, reference_course_deg=geometry.course,
                    observed_motion=task.get('target_motion', {}),
                    hull_radius_m=obs['target']['radius_m']),
        execution=dict({k: v for k, v in execution.items() if k not in (
            'fixed_dt_s', 'hull_length_m', 'hull_beam_m', 'obstacle_avoidance_radii_m')}, duration_s=duration),
        forecast_times_s=[duration]+times, candidate_schema=CANDIDATE_SCHEMA)
    if synchronize:
        state['heading_sync'] = dict(
            active=heading_active,
            current_error_deg=abs(course_error(geometry.course, execution['heading_course_deg'])))
    criteria = {}
    for name, (i, j) in ACTION_CHOICES.items():
        speed = obs['action_space']['speeds_knots'][i]*KNOT_MPS
        course = obs['action_space']['courses_deg'][j]
        motion = predict_ground_motion(boat['targetSpeed']*KNOT_MPS, speed, course,
                                       execution, duration, route_details=False)
        previews = [geometry.forecast(motion, duration)]
        for t in times:
            predicted = predict_ground_motion(boat['targetSpeed']*KNOT_MPS, speed, course,
                                              execution, t, dt=max(.1, execution['fixed_dt_s']), route_details=False)
            previews.append(geometry.forecast(predicted, t))
        criteria[name] = dict(speed_mps=speed, course_deg=course,
                              position_m=[[p['along_m'], p['across_m']] for p in previews],
                              band_error_m=[p['position_error_m'] for p in previews],
                              control_error_m=[p['control_error_m'] for p in previews],
                              control_gain_m=geometry.current['control_error_m']-previews[0]['control_error_m'],
                              end_rate_mps=[previews[0]['along_rate_mps'], previews[0]['across_rate_mps']],
                              preview_rate_mps=[previews[-1]['along_rate_mps'], previews[-1]['across_rate_mps']],
                              min_clearance_m=geometry.clearance(predicted, times[-1],
                                  max(.1, execution['fixed_dt_s']), boat['collision_radius_m']+obs['target']['radius_m']))
        if heading_active:
            criteria[name]['heading_error_deg'] = abs(course_error(geometry.target_at(duration)[1], motion['end_course_deg']))
        if guidance['course_deg'] is not None:
            criteria[name]['guidance_error_deg'] = [abs(course_error(guidance['course_deg'], course)),
                                                    abs(course_error(guidance['course_deg'], motion['end_course_deg']))]
    instructions = (ACQUIRE_INSTRUCTIONS if acquiring else POSITION_INSTRUCTIONS)
    instructions += ('Align heading only when heading_sync.active is true; otherwise position is the objective.'
                     if synchronize else 'Position only: there is no heading alignment objective.')
    instructions += (' Use guidance.course_deg for consistent steering: it accounts for the target turn and '
                     'gradual correction of position. Choose speed yourself using predicted progress, rates '
                     'and braking. Do not reverse a useful turn for tiny one-second position gains.')
    return dict(model=model, state=state, questions=dict(action=dict(
        type='choice', instructions=instructions,
        criteria=criteria)))


def build_stop_payload(obs, model='jev-latest'):
    payload = build_waypoint_stop_payload(obs, model)
    payload['state']['task']['background'] = 'Explicit STOP replaces accompaniment. Stop own boat; position and heading synchronization are suspended.'
    return payload


def status_params(context, observation=None):
    result = dict(targetPlatCode=context.get('targetPlatCode'))
    if observation is not None:
        geometry = AccompanyGeometry(observation)
        f = geometry.current
        result.update(currentAlongM=f['along_m'], currentAcrossM=f['across_m'], alongOutM=f['along_out_m'],
                      acrossOutM=f['across_out_m'], inPosition=int(f['in_position']),
                      targetSpeed=observation['target']['targetSpeed'], targetCourse=geometry.course)
    else:
        for key in ('currentAlongM', 'currentAcrossM', 'alongOutM', 'acrossOutM', 'inPosition', 'targetSpeed', 'targetCourse'):
            if key in context:
                result[key] = context[key]
    return result
