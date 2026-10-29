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
        def rounded(value):
            if isinstance(value, float):
                return round(value, 3)
            if isinstance(value, dict):
                return {k: rounded(v) for k, v in value.items()}
            if isinstance(value, list):
                return [rounded(v) for v in value]
            return value
        criteria = payload["questions"]["action"]["criteria"]
        if observation['task']['code'] == 'targetFollowing' and observation.get('operation') == 'execute':
            # Keep numeric facts lossless, but avoid repeating long column names 128 times.
            columns = [k for k in next(iter(criteria.values())) if k not in ('warnings', 'turn_error_deg')]
            payload['state']['candidate_value_columns'] = columns
            payload['state']['candidate_schema']['encoding'] = (
                'values is an ordered row using candidate_value_columns, in the displayed units. '
                'warnings and optional turn_error_deg remain named fields. All facts are retained.')
            for name, facts in criteria.items():
                row = dict(warnings=facts['warnings'], values=[facts[k] for k in columns])
                if 'turn_error_deg' in facts:
                    row['turn_error_deg'] = facts['turn_error_deg']
                criteria[name] = row
        for name, facts in criteria.items():
            if isinstance(facts, dict):
                criteria[name] = json.dumps(rounded(facts), ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        return payload
    return build
