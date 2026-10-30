"""Task 6: organize an incoming approach, then reach terminal state; 128 choices."""

import math

from action_space import (ACCOMPANY_ACTION_CHOICES as ACTION_CHOICES,
                          ACCOMPANY_COURSES_DEG as COURSES_DEG,
                          ACCOMPANY_DIRECTIONS as DIRECTIONS)
from geometry.coordinates import KNOT_MPS
from geometry.motion import predict_ground_motion, braking_facts, preview_duration
from geometry.relative_motion import course_error
from geometry.special_target_track import SpecialTrackGeometry


def validate_params(params):
    params.setdefault('expectedDistance', 1.0)
    if (type(params.get('targetBatch')) is not int or not 0 <= params['targetBatch'] <= 0xFFFFFFFF
            or any(type(params.get(k)) not in (int, float) or not math.isfinite(params[k])
                   for k in ('expectedAngleDeg', 'expectedSpeed', 'expectedDistance'))
            or not 0 <= params['expectedAngleDeg'] < 360 or params['expectedSpeed'] < 0
            or params['expectedDistance'] <= 0):
        raise ValueError('Invalid SpecialTargetTrackMode parameters')


def target_platform_id(params):
    # Batch identity is resolved against the complete CURRENT records, not cast to a platform ID.
    return None


def resolve_target_platform_id(params, records):
    matches = [r for r in records if r['platformId'] != params['platCode'] and r.get('batch') == params['targetBatch']]
    if len(matches) != 1:
        raise ValueError('Special target batch missing or ambiguous in current snapshot')
    return matches[0]['platformId']


def validate_manifest(params, radii, expected):
    if len(expected) != 2:
        raise ValueError('Special tracking supports own boat and one target only')


def adapt_observation(params, local, records, obs, state):
    pid = resolve_target_platform_id(params, records)
    target = next(r for r in records if r['platformId'] == pid)
    speed, course = target['targetSpeed'], target['targetCourse']
    if speed is None or not 0 <= speed <= 200 or speed > 0 and course is None:
        raise ValueError('Unknown special target motion')
    reference = course if course is not None else local['target_reference_course_deg']
    if type(reference) not in (int, float) or not math.isfinite(reference) or not 0 <= reference < 360:
        raise ValueError('Invalid target reference course')
    if params['expectedSpeed'] > local['max_speed_knots']:
        raise ValueError('Expected speed exceeds current own-boat capability')
    obs['target'] = dict(target, radius_m=local['collision_radii_m'][str(pid)])
    obs['task'] = dict(params, code='SpecialTargetTrackMode', reference_course_deg=reference)
    tick = local['tick']
    previous = state.get('special_motion')
    yaw = acceleration = 0.0
    if previous and tick > previous['tick']:
        dt = (tick-previous['tick'])*local['execution']['fixed_dt_s']
        if 0 < dt <= 5:
            alpha = dt/(.7+dt)
            yaw = previous['yaw']+alpha*(course_error(reference, previous['course'])/dt-previous['yaw'])
            acceleration = previous['acceleration']+alpha*((speed-previous['speed'])*KNOT_MPS/dt-previous['acceleration'])
    state['special_motion'] = dict(tick=tick, course=reference, speed=speed, yaw=yaw, acceleration=acceleration)
    obs['task']['target_motion'] = dict(yaw_deg_s=yaw if abs(yaw) >= .05 else 0,
                                       acceleration_mps2=acceleration if abs(acceleration) >= .15 else 0)
    geometry = SpecialTrackGeometry(obs)
    approach = geometry.approach()
    phase = state.get('special_phase', 'setup_approach')
    if phase == 'setup_approach' and approach['stage'] == 'terminal_approach':
        phase = 'terminal_approach'
    elif phase == 'terminal_approach' and (
            approach['incoming_axis_distance_m'] < -geometry.setup_length
            or abs(approach['incoming_axis_lateral_m']) > 2*geometry.setup_length):
        phase = 'setup_approach'
    state['special_phase'] = phase
    obs['task']['request_stage'] = phase


def validate_observation(obs, finite):
    validate_params(obs['task'])
    target = obs['target']
    return (not obs['obstacles'] and all(finite(target[k]) for k in ('longitude', 'latitude', 'targetSpeed', 'radius_m'))
            and -180 <= target['longitude'] <= 180 and -90 <= target['latitude'] <= 90
            and 0 <= target['targetSpeed'] <= 200 and target['radius_m'] > 0
            and target['batch'] == obs['task']['targetBatch']
            and (target['targetCourse'] is None and target['targetSpeed'] == 0
                 or finite(target['targetCourse']) and 0 <= target['targetCourse'] < 360)
            and finite(obs['task']['reference_course_deg']) and 0 <= obs['task']['reference_course_deg'] < 360
            and obs['task']['expectedSpeed'] <= max(obs['action_space']['speeds_knots']))


def build_payload(obs, model='jev-latest'):
    geometry = SpecialTrackGeometry(obs)
    boat, task, execution = obs['boat'], obs['task'], obs['execution']
    duration = obs['planning']['action_duration_s']
    speed = boat['targetSpeed']*KNOT_MPS
    horizon = preview_duration(execution, duration, speed)
    long_dt = max(.1, execution['fixed_dt_s'])
    approach = geometry.approach()
    approach['stage'] = task.get('request_stage', approach['stage'])
    terminal_guidance = geometry.terminal_guidance(horizon)
    criteria = {}
    for name, (i, j) in ACTION_CHOICES.items():
        command_speed = obs['action_space']['speeds_knots'][i]*KNOT_MPS
        command_course = obs['action_space']['courses_deg'][j]
        def predict(t, dt=None):
            return predict_ground_motion(speed, command_speed, command_course, execution, t,
                                         dt=dt, route_details=False)
        motion = predict(duration)
        end = geometry.at(motion['endpoint'], motion['end_speed'], motion['end_course_deg'], duration)
        future = predict(horizon, long_dt)
        long_clearance, closest = geometry.path_facts(future, horizon, long_dt)
        # FIRST capture freezes the experiment. If no capture, report the closest pass.
        t = closest['t_s']
        terminal_motion = predict(t, max(t, execution['fixed_dt_s'])) if t > 0 else None
        closest_facts = geometry.at(closest['position'], terminal_motion['end_speed'] if t > 0 else speed,
                                   terminal_motion['end_course_deg'] if t > 0 else execution['heading_course_deg'], t)
        target_end, end_course = geometry.target_at(duration)
        target_long, long_course = geometry.target_at(horizon)
        relative_end = tuple(geometry.relative[k]+motion['endpoint'][k]-target_end[k] for k in (0,1))
        relative_long = tuple(geometry.relative[k]+future['endpoint'][k]-target_long[k] for k in (0,1))
        criteria[name] = dict(command_speed_mps=command_speed, course_deg=command_course,
            end_position_error_m=end['position_error_m'], end_distance_m=end['distance_m'],
            end_bearing_error_deg=end['bearing_error_deg'], end_speed_error_mps=end['speed_error_mps'],
            end_pointing_error_deg=end['pointing_error_deg'],
            position_gain_m=geometry.current['position_error_m']-end['position_error_m'],
            pass_time_s=t, pass_is_capture=closest['is_capture'], pass_position_error_m=closest_facts['position_error_m'],
            pass_distance_m=closest_facts['distance_m'], pass_bearing_error_deg=closest_facts['bearing_error_deg'],
            pass_speed_error_mps=closest_facts['speed_error_mps'], pass_pointing_error_deg=closest_facts['pointing_error_deg'])
        if approach['stage']=='setup_approach':
            clearance, _ = geometry.path_facts(motion, duration, execution['fixed_dt_s'])
            criteria[name]['clearance_m'] = [clearance, long_clearance]
            criteria[name].update(setup_error_m=[geometry.setup_error(relative_end,end_course),geometry.setup_error(relative_long,long_course)])
            if approach['setup_guidance_course_deg'] is not None:
                criteria[name]['setup_guidance_error_deg'] = [abs(course_error(command_course,approach['setup_guidance_course_deg'])),
                    abs(course_error(motion['end_course_deg'],approach['setup_guidance_course_deg']))]
        else:
            criteria[name]['guidance_error_deg'] = [abs(course_error(command_course,terminal_guidance['course_deg'])),
                abs(course_error(motion['end_course_deg'],terminal_guidance['course_deg']))]
    state = dict(task=dict(mode='special_target_track', stage=approach['stage'],
        expected_bearing_deg=task['expectedAngleDeg'], expected_speed_mps=task['expectedSpeed']*KNOT_MPS,
        expected_distance_m=task['expectedDistance'],
        position_capture_radius_m=boat['collision_radius_m'],
        completion='Local experiment freezes on first entry into the position capture circle and records all errors; this is NOT four-condition acceptance.'),
        relative=geometry.current, boat=dict(actual_speed_mps=speed, heading_deg=execution['heading_course_deg']),
        approach=approach,
        guidance=terminal_guidance,
        target=dict(actual_speed_mps=geometry.speed, course_deg=geometry.course, observed_motion=task['target_motion']),
        execution={k:v for k,v in execution.items() if k not in ('fixed_dt_s','hull_length_m','hull_beam_m','obstacle_avoidance_radii_m')},
        action_duration_s=duration, preview_duration_s=horizon,
        braking=braking_facts(speed, execution, action_duration=duration),
        candidate_schema=dict(end='End fields are simultaneous at action_duration_s. position_gain_m >0 means progress to the moving terminal point.',
            passed_state='Pass fields share pass_time_s: FIRST sampled capture-circle entry if pass_is_capture=true, otherwise closest pass to the moving terminal point. Capture freezes the experiment. Not separate minima of four errors or four-condition acceptance.',
            setup='setup_error_m=[action end,held-preview end] distance to a moving setup point beyond the terminal point on its outward ray. Offset reserves relative closure during turning plus acceleration room, at least two nominal turn radii. setup_guidance_error_deg=[command,action-end actual] error to setup-point motion plus gradual position correction. Geometric aids, not selected commands or acceptance regions.',
            guidance='guidance_error_deg=[command,action-end actual] to guidance.course_deg, a line toward the terminal point predicted using observed target motion. It is steering reference only; Jev still chooses speed/course.',
            forecast='Bounded own acceleration/yaw; target follows observed turn, acceleration extrapolated at most 2s. Unknown future manoeuvres unavailable. Longer preview holds command; only one action is executed.'))
    instructions = (
        'Choose ONE command to reach a one-time terminal state: desired POSITION bearing relative to the target bow, '
        'desired centre distance, desired OWN ACTUAL speed, and own BOW pointing at the target, all at the SAME time. '
        'Bearing increases clockwise: 0 ahead, 90 starboard, 180 astern, 270 port. '
        'Do not confuse position bearing with the angle between bows. Do not follow or hold formation, and do not match target speed unless required by the terminal state. '
        'Arrange the approach to the specified side BEFORE final entry. You may turn away or temporarily increase position error to establish a useful approach. '
        'Align bow and adjust actual speed BEFORE entering the capture circle: entry freezes this local experiment, leaving no later correction. '
        'Use same-time candidate facts to coordinate position, speed and pointing; do not combine minima from different times. '
        'Maintain a useful turn when one-second progress is temporarily poor. Commands are target speeds and absolute north-clockwise courses, with bounded acceleration and yaw. '
        'Only the next action executes, then re-observe. Return one of the 128 choices.')
    if approach['stage']=='setup_approach':
        state['execution']['hull_length_m'] = execution['hull_length_m']
        state['candidate_schema']['clearance'] = ('Hull clearance values cover action / held preview, '
            'each ending at its FIRST capture or horizon. Motion after capture is excluded because Unity freezes. '
            '<=0 predicts contact before capture; includes moving target.')
        instructions = ('SETUP APPROACH. The objective of THIS decision is to actively close setup_error_m toward the moving '
            'setup point beyond the terminal point. Use approach.setup_guidance_course_deg for consistent steering; '
            'choose speed using predicted setup progress and acceleration limits. Do NOT aim the bow inward at the target '
            'yet, and do NOT limit speed to the eventual terminal speed while far from the setup point. '
            'Reach the setup area first so there is room to turn inward. The next request will handle final entry when '
            'incoming geometry permits it. Do not cut into the position capture circle from the wrong side or with an '
            'unprepared bow. A first-capture with large speed/pointing error freezes the experiment badly. '
            'A useful turn can temporarily worsen one-action progress; compare the held preview and reference deviation. '
            'Commands are target speed and absolute north-clockwise course, with bounded acceleration and yaw. '
            'Only one action executes before replanning. Return one of the 128 choices.')
        # This request has ONE navigation objective. Terminal fields otherwise made
        # Jev turn inward immediately while still behind the target in real tests.
        state['task'] = dict(mode='special_track_setup', stage='setup_approach',
            objective='reach_moving_setup_point',
            position_capture_radius_m=boat['collision_radius_m'],
            capture_warning='Entry into the terminal capture circle ends the experiment; prepare the incoming run first.')
        state.pop('relative')
        state.pop('guidance')
        for name, facts in criteria.items():
            capture = facts['pass_is_capture']
            criteria[name] = dict(command_speed_mps=facts['command_speed_mps'], course_deg=facts['course_deg'],
                setup_error_m=facts['setup_error_m'], setup_guidance_error_deg=facts.get('setup_guidance_error_deg'),
                capture_time_s=facts['pass_time_s'] if capture else None,
                capture_speed_error_mps=facts['pass_speed_error_mps'] if capture else None,
                capture_pointing_error_deg=facts['pass_pointing_error_deg'] if capture else None,
                clearance_m=facts['clearance_m'])
        state['candidate_schema'].pop('end')
        state['candidate_schema'].pop('passed_state')
        state['candidate_schema'].pop('guidance')
        state['candidate_schema']['capture'] = 'Capture fields are simultaneous at first predicted circle entry; null means no entry during held preview. They warn of premature capture; setup progress is the current objective.'
    else:
        state['candidate_schema'].pop('setup')
        state['approach'] = {k:v for k,v in approach.items()
                             if not k.startswith('setup_') and k != 'current_setup_error_m'}
        instructions += (' TERMINAL APPROACH: the incoming-ray geometry and available settling time support final approach. '
            'Steer consistently toward guidance.course_deg, comparing command and actual heading deviations; '
            'do not sail away from the terminal point merely because turning briefly increases one-step position error. '
            'Reach the moving terminal point with expected actual speed and bow pointing at target. '
            'Compare the first-capture speed and pointing errors together; prioritize their preparation over rushing entry.')
    return dict(model=model, state=state, questions=dict(action=dict(type='choice', instructions=instructions, criteria=criteria)))


def build_stop_payload(obs, model='jev-latest'):
    # Keep terminal semantics out of explicit STOP, including its prompt.
    execution = obs['execution']; duration = obs['planning']['action_duration_s']
    geometry = SpecialTrackGeometry(obs)
    speed = obs['boat']['targetSpeed']*KNOT_MPS
    criteria = {}
    for name, (i,j) in ACTION_CHOICES.items():
        v = obs['action_space']['speeds_knots'][i]*KNOT_MPS
        course = obs['action_space']['courses_deg'][j]
        motion = predict_ground_motion(speed, v, course, execution, duration, route_details=False)
        clearance, _ = geometry.path_facts(motion, duration, execution['fixed_dt_s'])
        criteria[name] = dict(command_speed_mps=v, course_deg=course, end_speed_mps=motion['end_speed'],
                              travel_m=math.hypot(*motion['endpoint']), clearance_m=clearance)
    return dict(model=model, state=dict(task=dict(stage='immediate_stop', stop_speed_mps=obs['task']['stop_speed_mps']),
        actual_speed_mps=speed, action_duration_s=duration, execution=execution,
        braking=braking_facts(speed, execution, action_duration=duration)),
        questions=dict(action=dict(type='choice', instructions='Explicit STOP replaces special tracking. Choose ONE zero-speed command to brake to rest, minimizing further travel. All 128 choices remain available.', criteria=criteria)))


def status_params(context, observation=None):
    result = dict(targetBatch=context.get('targetBatch'))
    if observation is not None:
        g = SpecialTrackGeometry(observation); f = g.current
        # Report the same (latched) phase used by the decision payload.
        stage = observation['task'].get('request_stage', g.approach()['stage'])
        result.update(relativeDistanceM=f['distance_m'], relativeBearingDeg=f['bearing_deg'],
                      angleErrorDeg=abs(f['bearing_error_deg']), ownSpeed=observation['boat']['targetSpeed'],
                      speedError=f['speed_error_mps']/KNOT_MPS, targetSpeed=observation['target']['targetSpeed'],
                      targetCourse=g.course, inTerminal=int(stage == 'terminal_approach'))
        result['unity_only'] = dict(distance_error_m=f['distance_error_m'], bearing_error_deg=f['bearing_error_deg'],
            pointing_error_deg=f['pointing_error_deg'], position_error_m=f['position_error_m'],
            position_capture_radius_m=observation['boat']['collision_radius_m'], four_conditions_accepted=False)
    else:
        for key in ('relativeDistanceM','relativeBearingDeg','angleErrorDeg','ownSpeed','speedError','targetSpeed','targetCourse','inTerminal'):
            if key in context: result[key] = context[key]
        if isinstance(context.get('unity_only'), dict): result['unity_only'] = dict(context['unity_only'])
    return result
