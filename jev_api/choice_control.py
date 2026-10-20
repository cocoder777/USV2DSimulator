"""64 speed/course choices. Geometry is descriptive; Jev selects the action."""
import math
from route_context import RouteContext

SPEED_NAMES = ('reverse_full', 'reverse_half', 'stop', 'forward_slow',
               'forward_quarter', 'forward_half', 'forward_three_quarters', 'forward_full')
HEADING_NAMES = ('ahead', 'ahead_left', 'left', 'behind_left', 'behind',
                 'behind_right', 'right', 'ahead_right')
ACTION_CHOICES = {f'{speed}__{heading}': {'speed_index': s, 'heading_index': h}
                  for s, speed in enumerate(SPEED_NAMES) for h, heading in enumerate(HEADING_NAMES)}

INSTRUCTIONS = (
    'Follow the ordered reference route closely while making forward progress through the CURRENT '
    'checkpoint. Coordinates are [forward metres, left metres] relative to the boat at sampling: '
    '[+,0] ahead, [0,+] LEFT, [0,-] RIGHT. The ordered ahead route points describe the upcoming '
    'dashed route, not separate checkpoints. Stay near that curve throughout the movement; '
    'do not shortcut bends just to reach a distant checkpoint. Enter the current checkpoint circle '
    'before proceeding to later route points. Choose ONE speed/course pair. '
    'This is NOT angular acceleration or a turn rate: heading changes INSTANTLY by the option angle '
    'relative to the CURRENT bow, once, then stays fixed for the whole interval. Positive angles '
    'turn LEFT; negative angles turn RIGHT. Speed is also set instantly with no inertia or damping. '
    'Each option states the resulting end position in CURRENT boat coordinates. '
    'Positive speed travels along the NEW bow; negative speed travels opposite the NEW bow. '
    'Prefer positive speed when equivalent movement is available. Compare the movement segment '
    'with the nearby route and current checkpoint. Use a smaller nonzero speed near sharp bends '
    'or when a full step would overshoot. Do not choose stop merely because you are on the route '
    'or inside an intermediate checkpoint; continue making progress. '
    'Judge LEFT versus RIGHT using the signs in the CURRENT coordinate frame, even after the last '
    'action changed the boat heading. For example, nearby route points [1,0],[2,0] call for ahead '
    'motion, NOT another turn; [1,1],[2,2] call for ahead-LEFT, while [1,-1],[2,-2] call for ahead-RIGHT. '
    'Follow the NEAR portion (roughly the next 1-2 metres) first; the far checkpoint does not justify '
    'departing from that portion. If already close to the route, do not create a large sideways '
    'deviation. When the curve is between the available directions, alternate short movements '
    'rather than long zigzags. Prefer smaller displacement over a move that leaves the curve '
    'by about a metre. At the final target, stop immediately if already inside the arrival circle; '
    'there is no need to reach its exact centre. '
)


def build_payload(observation, model='jev-latest', context=None, run_key=None):
    context = context or RouteContext()
    state = context.prepare(observation, run_key)
    duration = observation['planning']['action_duration_s']
    state['action_duration_s'] = duration
    task = observation['task']
    final = task['next_waypoint_index'] == task['planWaypointNum'] - 1
    goal = ('The CURRENT target is FINAL: finish inside its circle AND with abs(speed) <= '
            f"{task['stop_speed_mps']} m/s. Approach at a useful speed; choose stop once inside. "
            'If outside, select a movement which ends inside the circle, then stop next decision.'
            if final else 'The CURRENT target is an intermediate CHECKPOINT: entering its circle is sufficient; no stop is required.')
    criteria = {}
    for name, action in ACTION_CHOICES.items():
        speed = observation['action_space']['speeds_mps'][action['speed_index']]
        angle = observation['action_space']['heading_offsets_deg'][action['heading_index']]
        forward = speed * duration * math.cos(math.radians(angle))
        left = speed * duration * math.sin(math.radians(angle))
        forward = 0.0 if abs(forward) < 0.0005 else forward
        left = 0.0 if abs(left) < 0.0005 else left
        if speed == 0:
            situation = 'STOP: suitable only when already inside the final parking circle.'
        else:
            direction_index = (action['heading_index'] + (4 if speed < 0 else 0)) % 8
            direction = HEADING_NAMES[direction_index].replace('_', ' ').upper()
            situation = f'Moves {direction} in the CURRENT frame. Use when the NEAR route continues in that direction.'
        criteria[name] = (f'Set speed={speed:.4g} m/s; new bow offset={angle:+g} deg. '
                          f'End position [forward,left]=[{forward:.3f},{left:.3f}] m. '
                          f'{situation} Travel distance={abs(speed)*duration:.3f} m. '
                          'This movement must stay near the route throughout the interval.')
    return {'model': model, 'state': state, 'questions': {'action': {
        'type': 'choice', 'instructions': INSTRUCTIONS + goal, 'criteria': criteria}}}


def decode_action(response):
    answers = response.get('answers') if isinstance(response, dict) else None
    answer = answers.get('action') if isinstance(answers, dict) else None
    if not isinstance(answer, dict) or answer.get('type') != 'choice':
        raise ValueError('Expected Choice answer for action')
    name = answer.get('choice')
    if not isinstance(name, str) or name not in ACTION_CHOICES:
        raise ValueError('Unknown speed/course choice')
    return dict(ACTION_CHOICES[name])
