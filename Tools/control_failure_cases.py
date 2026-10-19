"""Independent, predeclared offline control probes; never dispatches boat commands."""
import copy
import itertools
import json
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'jev_api'))
from choice_control import ACTION_CHOICES, build_payload
from classic_control import action_values, pd_action, predict_action

SEED = 20261018
FOUR_SEED = 20261021  # First subsequent seed with six unique non-original orders for K=4.
KEYS = list(ACTION_CHOICES)
FIXTURE = ROOT / 'Logs/control-benchmark-20261018-classic-pd-v2/worker.jsonl'
# Same observation except explicitly listed speed, yaw, target and stop condition.
STATES = [
    ('ahead_rest', 0, 0, (8, 0), False),
    ('ahead_reverse', -1, 0, (8, 0), False),
    ('ahead_forward', 1, 0, (8, 0), False),
    ('near_fast', 3, 0, (1.5, 0), True),
    ('left_rest', 0, 0, (8, 5), False),
    ('right_rest', 0, 0, (8, -5), False),
    ('yaw_left', 1.5, 30, (10, 0), False),
    ('yaw_right', 1.5, -30, (10, 0), False),
    ('inside_forward', .3, 0, (.2, 0), True),
    ('inside_reverse', -.3, 0, (.2, 0), True),
    ('inside_rest', 0, 0, (.2, 0), True),
    ('inside_near_zero', .05, 0, (.2, 0), True),
]


def observations():
    first = json.loads(FIXTURE.read_text(encoding='utf-8').splitlines()[0])
    base = first['request']['observation']
    for name, v, w, xy, stop in STATES:
        obs = copy.deepcopy(base)
        obs['boat'].update(speed_mps=v, yaw_rate_deg_s=w)
        obs['target'].update(position_xy_m=list(xy), requires_stop=stop)
        yield name, obs


def acceptable(name, obs, action, scope='joint'):
    """Necessary directional tests, not an optimal-action oracle or navigation success.

    Axis-only tests evaluate only that axis. Near parking permits damping/coasting
    if it reduces speed; exact agreement with PD is never a requirement.
    """
    a, t = action['acceleration_index'], action['turn_index']
    linear, angular = action_values(obs, a, t)
    pred = predict_action(obs, a, t)
    v, w = obs['boat']['speed_mps'], obs['boat']['yaw_rate_deg_s']
    if name.startswith('inside'):
        lin = abs(pred.speed_mps) <= .1 if abs(v) <= .1 else abs(pred.speed_mps) < abs(v)
    elif name == 'near_fast':
        lin = linear < 0  # direction only: this deliberately severe state may overshoot
    elif name in ('ahead_rest', 'ahead_reverse'):
        lin = linear > 0
    elif name == 'ahead_forward':
        lin = linear >= 0
    elif name in ('left_rest', 'right_rest'):
        lin = linear >= 0  # turning in place is allowed
    else:
        lin = True  # yaw-only counterfactual makes no claim about thrust optimality
    if name == 'left_rest':
        turn = angular > 0
    elif name == 'right_rest':
        turn = angular < 0
    elif name.startswith('yaw_'):
        turn = angular * w <= 0 and abs(pred.yaw_rate_deg_s) < abs(w)
    else:
        # No terminal-heading requirement exists. For an aligned target a nonzero
        # angular input is needless, but one offline step cannot prove it makes
        # navigation impossible. Keep straight coasting as a separate preference.
        turn = True
    return {'linear': lin, 'angular': turn,
            'straight_preference': angular == 0 if name not in ('left_rest','right_rest','yaw_left','yaw_right') else None,
            'reasonable': lin if scope == 'linear8' else turn if scope == 'angular7' else lin and turn}


def indices(name, family):
    if family == 'joint56':
        return list(range(56))
    if family == 'linear8':
        return [a * 7 + 3 for a in range(8)]
    if family == 'angular7':
        return [21 + t for t in range(7)]
    # Four interpretable controls, predeclared by probe group, never selected by PD.
    if name in ('left_rest', 'right_rest'):
        return [24, 52, 28, 34]  # coast, forward, gentle-forward/right, gentle-forward/left
    if name.startswith('yaw_'):
        return [24, 52, 21, 27]  # coast, forward, pure right, pure left
    return [3, 24, 31, 52]  # backward, coast, gentle forward, full forward


def payload(name, obs, family, style, backend):
    profile = 'compact' if backend == 'laya' else 'original'
    p = build_payload(obs, 'multilingual' if backend == 'laya' else 'kev-latest', profile)
    selected = indices(name, family)
    criteria, mapping = {}, {}
    for i in selected:
        action = ACTION_CHOICES[KEYS[i]]
        a, t = action_values(obs, **action)
        if style == 'short':
            key, desc = f'a={a:.4g},alpha={t:.4g}', ''
        elif style == 'brief':
            key = f'k{i:02}'
            linear = 'forward acceleration' if a > 0 else 'backward acceleration' if a < 0 else 'coast'
            turn = 'left acceleration' if t > 0 else 'right acceleration' if t < 0 else 'zero angular input'
            desc = f'{linear} {a:.4g} m/s^2; {turn} {t:.4g} deg/s^2.'
        else:
            key, desc = KEYS[i], p['questions']['action_0']['criteria'][KEYS[i]]
        criteria[key] = desc
        mapping[key] = {'physical_id': i, 'action': action, 'vector': [a, t]}
    p['questions']['action_0']['criteria'] = criteria
    if family == 'linear8':
        p['questions']['action_0']['instructions'] += ' This probe asks only for longitudinal acceleration; ignore the fixed angular component.'
    if family == 'angular7':
        p['questions']['action_0']['instructions'] += ' This probe asks only for angular acceleration; ignore the fixed longitudinal component.'
    if backend == 'laya':
        p.update(max_len=4096, head_max_len=3072)
    assert any(acceptable(name, obs, x['action'], family)['reasonable'] for x in mapping.values())
    return p, mapping


def orders(keys):
    rng = random.Random(FOUR_SEED if len(keys) == 4 else SEED)
    result = [list(keys)]
    # Matches Kev official permute implementation: original plus SIX random orders.
    for _ in range(6):
        order = list(keys)
        rng.shuffle(order)
        result.append(order)
    return result


def evaluate(name, obs, family, mapping, answer):
    chosen = mapping[answer['choice']]
    pd = pd_action(obs)
    pd_vector = action_values(obs, **pd)
    return {'selected': chosen, 'pd_action': pd, 'pd_vector': pd_vector,
            'delta_pd': [x-y for x, y in zip(chosen['vector'], pd_vector)],
            'checks': acceptable(name, obs, chosen['action'], family),
            'top3': [{'key': k, 'p': v, **mapping[k]} for k, v in
                     sorted(answer['probabilities'].items(), key=lambda kv: -kv[1])[:3]]}


def summarize(rows):
    groups = {}
    for row in rows:
        groups.setdefault((row['family'], row['style']), []).append(row)
    result = []
    for (family, style), group in groups.items():
        valid = [r for r in group if not r.get('error')]
        assessed = [r for r in valid if
                    (family != 'angular7' or r['state_name'] in ('left_rest','right_rest','yaw_left','yaw_right')) and
                    (family != 'linear8' or not r['state_name'].startswith('yaw_'))]
        cases = {}
        for r in valid:
            cases.setdefault(r['state_name'], []).append(r)
        pairs = flips = unstable = 0
        for runs in cases.values():
            unstable += len({r['evaluation']['selected']['physical_id'] for r in runs}) > 1
            for a, b in itertools.combinations(runs, 2):
                pairs += 1
                flips += a['evaluation']['selected']['physical_id'] != b['evaluation']['selected']['physical_id']
        result.append({'family': family, 'style': style, 'valid': len(valid),
                       'errors': len(group)-len(valid),
                       'evaluated': len(assessed),
                       'reasonable': sum(r['evaluation']['checks']['reasonable'] for r in assessed),
                       'negative_linear': sum(r['evaluation']['selected']['vector'][0] < 0 for r in valid),
                       'unstable_states': unstable, 'states': len(cases),
                       'pairwise_flip_rate': flips/pairs if pairs else None})
    return result
