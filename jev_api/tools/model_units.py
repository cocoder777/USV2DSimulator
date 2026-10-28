"""Jev 展示单位转换；预测、风险布尔值及 Choice 映射均不改变。"""

from functools import wraps
import json
import re
from geometry.coordinates import KNOT_MPS


def model_speed_units(builder):
    @wraps(builder)
    def build(observation, *args, **kwargs):
        payload = builder(observation, *args, **kwargs)
        def convert(value):
            if isinstance(value, dict):
                return {
                    (k[:-4] + "_knots" if k.endswith("_mps") else k):
                    (None if v is None else round(v / KNOT_MPS, 6))
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
        for name, facts in criteria.items():
            if isinstance(facts, dict):
                criteria[name] = json.dumps(rounded(facts), ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        return payload
    return build
