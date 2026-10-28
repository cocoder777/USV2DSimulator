"""Task 3: no-obstacle accompaniment; two explicit request variants."""

import math

from action_space import ACTION_CHOICES, SPEED_FRACTIONS
from geometry.accompany import AccompanyGeometry
from geometry.coordinates import KNOT_MPS
from geometry.motion import predict_ground_motion
from geometry.relative_motion import course_error
from tasks.stop import build_waypoint_stop_payload

BOUNDS = ('offsetAlongMinM', 'offsetAlongMaxM', 'offsetAcrossMinM', 'offsetAcrossMaxM')
POSITION_INSTRUCTIONS = (
    'Choose ONE speed/course command to ENTER and MAINTAIN the allowed position rectangle relative to the '
    'friendly target. Along is positive FORWARD, negative BEHIND; across is positive STARBOARD/right, '
    'negative PORT/left of target direction. First reduce position error to the RECTANGLE, then remain '
    'anywhere inside it; its centre is not a mandatory point. Compare two consequences: end position error '
    '(entry/recovery), and end relative along/across velocity (stable holding). When far outside, make '
    'useful progress; as you approach, reduce relative velocity early to avoid overshooting. Do not match '
    'target velocity while outside and freeze the position error. Inside the rectangle, keep relative '
    'velocity small; being inside for one action does not finish this continuous task. The boat starts '
    'outside and must enter by its own movement. Avoid contact with the friendly target hull. No other '
    'obstacles are present. Target future turns/stops are unknown; predict constant current '
    'velocity/direction for this action only. Commands are target speeds and ABSOLUTE courses clockwise '
    'from north, not instantaneous velocity changes. Use measured heading/yaw and bounded motion: turning '
    'takes time; zero target speed brakes and ignores requested course. Keep useful turns consistent rather'
    ' than reversing every second for tiny position gains. All original 64 choices are available; Python '
    'never ranks, filters or substitutes an action. '
)
NO_SYNC_INSTRUCTIONS = POSITION_INSTRUCTIONS + (
    'This is POSITION ONLY: there is NO heading alignment objective. Select whatever heading helps enter '
    'and hold the rectangle. '
)
SYNC_INSTRUCTIONS = POSITION_INSTRUCTIONS + (
    'This is POSITION AND HEADING SYNCHRONIZATION: once position is acquired and stable, align own bow with'
    ' target direction. During entry a different heading is necessary and allowed. Prioritize '
    'acquiring/holding the rectangle over premature alignment. Only eight absolute courses exist: '
    'approximate alignment with available courses, making small corrections to hold position. Use candidate'
    ' end heading_error_deg as the additional alignment fact. '
)
CANDIDATE_SCHEMA = {'position': 'along_m/across_m: action END own position in the target frame, metres.',
 'error': 'along_out_m/across_out_m: signed distance outside each interval; zero inside. position_error_m: '
          'Euclidean distance to rectangle.',
 'holding': 'relative_along/relative_across: action END own minus target velocity in target axes. Zero means '
            'stable relative position.',
 'forecast': 'One action only; target velocity and reference direction remain constant. Predictions include '
             'own acceleration and yaw inertia.'}


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
    synchronize = task['request_variant'] == 'heading_sync'
    state = dict(
        task=dict(mode='accompany', objective='position_and_heading' if synchronize else 'position_only',
                  allowed_along_m=[task['offsetAlongMinM'], task['offsetAlongMaxM']],
                  allowed_across_m=[task['offsetAcrossMinM'], task['offsetAcrossMaxM']]),
        relative=geometry.current,
        boat=dict(actual_speed_mps=boat['targetSpeed']*KNOT_MPS,
                  heading_deg=execution['heading_course_deg'], yaw_deg_s=execution['yaw_rate_deg_s'],
                  hull_radius_m=boat['collision_radius_m']),
        target=dict(actual_speed_mps=geometry.target_speed, reference_course_deg=geometry.course,
                    hull_radius_m=obs['target']['radius_m']),
        execution=dict(execution, duration_s=duration), candidate_schema=CANDIDATE_SCHEMA)
    if synchronize:
        state['heading_sync'] = dict(
            current_error_deg=abs(course_error(geometry.course, execution['heading_course_deg'])),
            instruction='Align after acquiring stable position; approximate with eight courses.')
    criteria = {}
    for name, (i, j) in ACTION_CHOICES.items():
        speed = obs['action_space']['speeds_knots'][i]*KNOT_MPS
        course = obs['action_space']['courses_deg'][j]
        motion = predict_ground_motion(boat['targetSpeed']*KNOT_MPS, boat['targetCourse'], speed, course,
                                       execution, duration, route_details=False)
        criteria[name] = dict(speed_mps=speed, course_deg=course,
                              **geometry.candidate(motion, duration, synchronize))
    return dict(model=model, state=state, questions=dict(action=dict(
        type='choice', instructions=SYNC_INSTRUCTIONS if synchronize else NO_SYNC_INSTRUCTIONS,
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
