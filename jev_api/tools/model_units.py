"""Jev 展示单位转换；预测、风险布尔值及 Choice 映射均不改变。"""

from functools import wraps
import re
from tools.waypoint_geometry import KNOT_MPS


def model_speed_units(builder):
    @wraps(builder)
    def build(observation, *args, **kwargs):
        payload = builder(observation, *args, **kwargs)
        if observation.get("model_speed_unit", "knots") == "mps" or "units" in payload["state"]:
            return payload

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

        payload = convert(payload)
        payload["state"]["units"] = {
            "speed": "knots", "distance": "metres",
            "conversion": "1 knot = 1852/3600 m/s = approximately 0.514444 m/s."
        }
        return payload
    return build
