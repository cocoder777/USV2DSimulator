"""Jev 展示单位转换；预测、风险布尔值及 Choice 映射均不改变。"""

from functools import wraps
import json
import re
from geometry.coordinates import KNOT_MPS


def model_speed_units(builder):
    @wraps(builder)
    def build(observation, *args, **kwargs):
        payload = builder(observation, *args, **kwargs)
        def convert_speed(value):
            if isinstance(value, list):
                return [convert_speed(item) for item in value]
            return None if value is None else round(value / KNOT_MPS, 6)
        def convert(value):
            if isinstance(value, dict):
                return {
                    (k[:-4] + "_knots" if k.endswith("_mps") else k):
                    convert_speed(v)
                    if k.endswith("_mps") else convert(v)
                    for k, v in value.items()
                }
            if isinstance(value, list):
                return [convert(v) for v in value]
            if isinstance(value, str):
                # Existing candidate renderers emit numeric speed facts in m/s.
                # Only their displayed values change; calculations already finished.
                return re.sub(r"(-?\d+(?:\.\d+)?) m/s\b",
                              lambda m: f"{float(m[1]) / KNOT_MPS:.3f} knots", value)
            return value

        if observation.get("model_speed_unit", "knots") != "mps" and "units" not in payload["state"]:
            payload = convert(payload)
            payload["state"]["units"] = {
                "speed": "knots", "distance": "metres",
                "conversion": "1 knot = 1852/3600 m/s = approximately 0.514444 m/s."
            }
        # Compact following facts stay structured until units are converted.
        # Jev's Choice API still receives strings, with the original choice IDs.
        def rounded(value, digits=3):
            if isinstance(value, float):
                return round(value, digits)
            if isinstance(value, dict):
                return {k: rounded(v, digits) for k, v in value.items()}
            if isinstance(value, list):
                return [rounded(v, digits) for v in value]
            return value
        criteria = payload["questions"]["action"]["criteria"]
        if observation['task']['code'] == 'followMode' and observation.get('operation') == 'execute':
            state = payload['state']
            compact = lambda value: json.dumps(rounded(value, 1), allow_nan=False, separators=(',', ':'))
            for name, facts in criteria.items():
                unit, speed_key = ('knots', 'speed_knots') if 'speed_knots' in facts else ('m/s', 'speed_mps')
                safety = facts['safety_clearance_m']
                first, last = state['forecast_times_s']
                criteria[name] = (
                    f"{facts[speed_key]:.3f} {unit}, course {facts['course_deg']:g} deg. "
                    f"Point error {first:g}/{last:g}s {compact(facts['tracking_error_m'])}m. "
                    + (f"Reference deviation command/first {compact(facts['guidance_error_deg'])}deg. " if 'guidance_error_deg' in facts else '')
                    + f"Hull clearance action/long {compact(facts['hull_clearance_m'][::2])}m. "
                    + f"Safety clearance min/end action/long {compact(safety)}m. "
                    + ('SAFETY INTRUSION IN ACTION.' if safety[0] < 0 else 'SAFETY INTRUSION IF HELD.' if safety[2] < 0 else 'Safety clear.')
                )
            state['candidate_schema'] = {
                'tracking': 'Point error at forecast_times_s is distance to ONE current-direction rear point minus tolerance, floored at zero. No heading alignment objective.',
                'clearance': 'Hull and safety clearance cover BOTH the moving target and every nearby obstacle. Target safety radius=max(minimum following distance,combined hulls); obstacle safety uses its avoidance circle. Hull arrays=[action min,long min]; safety=[action min,end,long min,end]. Safety<0=intrusion; hull<=0=contact. Minima include shared start; endpoints allow recovery. Ordinary obstacles frozen at current positions.',
                'reference': 'Heading errors command/actual first are to the position-tracking course reference, accounting for current target velocity and gradual point-error correction. It is not an executable command. All 128 choices remain available.',
            }
        elif observation['task']['code'] == 'followMode':
            columns = list(next(iter(criteria.values())))
            payload['state']['candidate_value_columns'] = columns
            for name, facts in criteria.items():
                # Decimetre/0.1-degree display keeps large 128-choice requests
                # bounded. Predictions, threshold flags and decoded controls
                # retain their original precision; speed display stays at 3dp.
                row = [rounded(facts[k], 1 if k.endswith(('_m', '_deg')) else 3) for k in columns]
                criteria[name] = json.dumps(rounded(row), ensure_ascii=False,
                                            allow_nan=False, separators=(",", ":"))
        for name, facts in criteria.items():
            if isinstance(facts, dict):
                criteria[name] = json.dumps(rounded(facts), ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        return payload
    return build
