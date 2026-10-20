"""64 speed/course choices. Geometry is descriptive; Jev selects the action."""
import math
from route_context import RouteContext
from navigation_motion import delta_angle, predict_motion

SPEED_NAMES = ('reverse_full', 'reverse_half', 'stop', 'forward_slow',
               'forward_quarter', 'forward_half', 'forward_three_quarters', 'forward_full')
HEADING_NAMES = ('ahead', 'ahead_left', 'left', 'behind_left', 'behind',
                 'behind_right', 'right', 'ahead_right')
HEADING_SITUATIONS = (
    'Near route lies AHEAD and little heading correction is needed, or residual yaw must be braked. Poor when the route is behind or far to either side.',
    'Near route lies AHEAD-LEFT; steer left and account for current yaw.',
    'Near route lies mainly LEFT; make a substantial left turn, usually at low speed.',
    'Current checkpoint or recovery route is BEHIND-LEFT; turn back left while limiting forward travel away.',
    'Current checkpoint or recovery route is BEHIND; turn back rather than continue away. Use low speed during the turn.',
    'Current checkpoint or recovery route is BEHIND-RIGHT; turn back right while limiting forward travel away.',
    'Near route lies mainly RIGHT; make a substantial right turn, usually at low speed.',
    'Near route lies AHEAD-RIGHT; steer right and account for current yaw.',
)
SPEED_SITUATIONS = (
    'Strong reverse target for deliberate backward recovery; not for normal forward progress.',
    'Gentle reverse target for small backward corrections.',
    'Zero target to park or reduce speed before a substantial turn; avoid staying stopped outside the final circle.',
    'Small forward target for precise curved corrections, sharp turns, or final approach.',
    'Low forward target for following bends and correcting route deviation.',
    'Moderate forward target for progress when the near route is reasonably aligned.',
    'Fast forward target only when near route is mostly ahead, yaw is controlled, and the movement stays close to the curve.',
    'Maximum forward target ONLY for an aligned near route ahead. Poor for large route deviation, a checkpoint behind, or sharp turning.',
)
ACTION_CHOICES = {f'{speed}__{heading}': {'speed_index': s, 'heading_index': h}
                  for s, speed in enumerate(SPEED_NAMES) for h, heading in enumerate(HEADING_NAMES)}

INSTRUCTIONS = (
    'Follow the ordered reference route closely while making forward progress through the CURRENT '
    'checkpoint. Coordinates are [forward metres, left metres] relative to the boat at sampling: '
    '[+,0] ahead, [0,+] LEFT, [0,-] RIGHT. The ordered ahead route points describe the upcoming '
    'dashed route, not separate checkpoints. Stay near that curve throughout the movement; '
    'do not shortcut bends just to reach a distant checkpoint. Enter the current checkpoint circle '
    'before proceeding to later route points. Choose ONE speed/course pair. '
    'Each option specifies a TARGET heading offset relative to the CURRENT sampled bow, once. '
    'It is not an angular acceleration or turn-rate command. Positive offsets mean LEFT, negative RIGHT. '
    'Speed and heading follow the response rules below. '
    'Each option states the resulting end position in CURRENT boat coordinates. '
    'Positive speed moves along the INSTANTANEOUS bow, negative opposite it, while the bow may still be turning. '
    'Prefer positive speed when equivalent movement is available. Compare the movement segment '
    'with the nearby route and current checkpoint. Use a smaller nonzero speed near sharp bends '
    'or when a full step would overshoot. Do not choose stop merely because you are on the route '
    'or inside an intermediate checkpoint; continue making progress. '
    'Judge LEFT versus RIGHT using the signs in the CURRENT coordinate frame, even after the last '
    'action changed the boat heading. For example, nearby route points [1,0],[2,0] call for ahead '
    'motion; account for residual yaw before selecting another turn. [1,1],[2,2] indicate ahead-LEFT, '
    'while [1,-1],[2,-2] indicate ahead-RIGHT. '
    'Follow the NEAREST supplied ahead route points first; the far checkpoint does not justify '
    'departing from that portion. If already close to the route, do not create a large sideways '
    'deviation. When the curve is between the available directions, alternate short movements '
    'rather than long zigzags. Prefer smaller displacement when it reduces departure from the curve. '
    'At the final target, target zero speed if already inside the arrival circle; '
    'there is no need to reach its exact centre. '
    'If target.forward_left_m[0] is negative, that checkpoint is BEHIND the sampled bow. '
    'Slow down and turn back toward it; never keep charging ahead away from an unpassed checkpoint. '
    'A large cross_track_distance_m calls for recovery toward the near route, not maximum-speed forward travel. '
)


def speed_response(initial, target, transition_time, duration):
    """Exact signed distance and end speed for a held target, not an action recommendation."""
    if transition_time <= 0:
        return target * duration, target
    ramp_time = min(duration, transition_time)
    acceleration = (target - initial) / transition_time
    distance = initial * ramp_time + 0.5 * acceleration * ramp_time ** 2
    end_speed = target if duration >= transition_time else initial + acceleration * ramp_time
    return distance + target * (duration - ramp_time), end_speed


def build_payload(observation, model='jev-latest', context=None, run_key=None):
    context = context or RouteContext()
    state = context.prepare(observation, run_key)
    duration = observation['planning']['action_duration_s']
    state['action_duration_s'] = duration
    task = observation['task']
    transition_duration = observation['response']['speed_transition_time_s']
    initial_speed = observation['boat']['speed_mps']
    if transition_duration > 0:
        state['boat'] = {'speed_mps': initial_speed}
        state['speed_response'] = dict(observation['response'])
        response_rule = (
            'A CHANGED target speed starts CONSTANT ACCELERATION immediately: '
            'a=(target_speed-current_speed)/speed_transition_time_s. It reaches the target '
            'exactly after speed_transition_time_s of SIMULATION time, then holds it. '
            'There is NO waiting delay before acceleration and no exponential response or damping. '
            'Keeping active_target_speed_mps continues the existing ramp for remaining_transition_s; '
            'it does NOT restart the transition clock. A different target starts a new full-duration ramp. '
            'Current signed speed is boat.speed_mps. A stop option targets zero but DOES NOT instantly stop motion. '
            'Each option gives the integrated end position and end speed, accounting for this lag. '
            'When reversing signed speed, motion may first continue in the old signed direction. '
            'Use smaller target speeds before bends and the final circle. To park, reduce speed '
            'before arrival and target zero early enough to remain inside the circle while slowing. '
            'While inside the final circle with residual speed, continue targeting zero; do not '
            'add thrust just because completion needs another interval. '
        )
    else:
        response_rule = 'Speed is set INSTANTLY with no inertia or damping. Stop sets actual speed to zero immediately. '
    heading_response = observation['heading_response']
    if heading_response['transition_time_s'] > 0:
        state.setdefault('boat', {})['yaw_rate_deg_s'] = observation['boat']['yaw_rate_deg_s']
        state['heading_response'] = dict(heading_response)
        response_rule += (
            'Heading changes GRADUALLY with CONTINUOUS yaw rate. A changed target uses the shortest '
            'signed heading difference and a cubic curve starting at the CURRENT yaw rate and '
            'ending at the target heading with ZERO yaw after heading_response.transition_time_s. '
            'This is an immediate angular-acceleration process, not a waiting delay or instant rotation. '
            'The current yaw can carry the bow '
            'briefly the old way before reversing. Keeping the same WORLD target heading continues '
            'its remaining_transition_s. Relative option offsets refer to the sampled bow, not the '
            'previous target, so repeatedly choosing a nonzero offset can keep moving the target. '
            'Motion during turning is CURVED: use each option midpoint, endpoint and final yaw. '
            'AHEAD targets the sampled heading; with residual yaw it brakes and returns toward that heading. '
        )
    else:
        response_rule += 'Heading is set instantly to the option target and yaw rate becomes zero. '
    final = task['next_waypoint_index'] == task['planWaypointNum'] - 1
    if final:
        distance = math.hypot(*state['target']['forward_left_m'])
        state['target']['distance_m'] = round(distance, 3)
        state['target']['inside_arrival_circle'] = distance <= task['arrival_radius_m']
    goal = ('The CURRENT target is FINAL: finish inside its circle AND with abs(speed) <= '
            f"{task['stop_speed_mps']} m/s. Approach at a useful speed; choose stop once inside. "
            'target.inside_arrival_circle explicitly tells whether the location condition is met. '
            'If FALSE and speed is already small, ZERO target speed cannot finish the task: '
            'choose a SMALL NONZERO movement toward the target to ENTER the circle. '
            'Distance greater than target.arrival_radius_m means OUTSIDE, not arrived. '
            'If outside with substantial speed, slow early enough to remain inside when stopping. '
            'Once inside, target zero until the actual speed satisfies the threshold.'
            if final else 'The CURRENT target is an intermediate CHECKPOINT: entering its circle is sufficient; no stop is required.')
    criteria = {}
    for name, action in ACTION_CHOICES.items():
        speed = observation['action_space']['speeds_mps'][action['speed_index']]
        angle = observation['action_space']['heading_offsets_deg'][action['heading_index']]
        transition_time = (observation['response']['remaining_transition_s']
                           if speed == observation['response']['active_target_speed_mps'] else transition_duration)
        same_heading = abs(delta_angle(angle-heading_response['active_target_offset_deg'])) <= .0001
        turn_time = (heading_response['remaining_transition_s'] if same_heading
                     else heading_response['transition_time_s'])
        turn_delta = heading_response['active_target_offset_deg'] if same_heading else angle
        motion = predict_motion(initial_speed, speed, transition_time, turn_delta,
                                observation['boat']['yaw_rate_deg_s'], turn_time,
                                duration, observation['fixed_dt_s'])
        forward, left = motion['endpoint']
        situation = (SPEED_SITUATIONS[2 if speed == 0 else action['speed_index']] + ' Heading situation: '
                     + HEADING_SITUATIONS[action['heading_index']]
                     + ' BOTH situations must fit. Negative speed moves opposite the turning bow.')
        mx,my = motion['midpoint']
        criteria[name] = (f'Target speed={speed:.4g} m/s; target bow offset={angle:+g} deg. '
                          f'Midpoint [forward,left]=[{mx:.3f},{my:.3f}] m; '
                          f'End position [forward,left]=[{forward:.3f},{left:.3f}] m. '
                          f"End signed speed={motion['end_speed']:.3f} m/s; "
                          f"end bow offset={motion['end_heading']:.3f} deg; end yaw={motion['end_yaw']:.3f} deg/s. "
                          f'{situation} Stay near the route throughout the interval.')
    return {'model': model, 'state': state, 'questions': {'action': {
        'type': 'choice', 'instructions': INSTRUCTIONS + response_rule + goal, 'criteria': criteria}}}


def decode_action(response):
    answers = response.get('answers') if isinstance(response, dict) else None
    answer = answers.get('action') if isinstance(answers, dict) else None
    if not isinstance(answer, dict) or answer.get('type') != 'choice':
        raise ValueError('Expected Choice answer for action')
    name = answer.get('choice')
    if not isinstance(name, str) or name not in ACTION_CHOICES:
        raise ValueError('Unknown speed/course choice')
    return dict(ACTION_CHOICES[name])
