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
                    + (f"Region error {compact(facts['region_error_m'])}m. " if 'region_error_m' in facts else f"Point error {compact(facts['tracking_error_m'])}m. ")
                    + (f"Reference deviation command/first {compact(facts['guidance_error_deg'])}deg. " if 'guidance_error_deg' in facts else '')
                    + (f"Bypass remaining {compact(facts['bypass_remaining_m'])}m. " if 'bypass_remaining_m' in facts else '')
                    + (f"If target straightens, region error {compact(facts['coast_region_error_m'])}m. " if 'coast_region_error_m' in facts else '')
                    + f"Hull clearance action/long {compact(facts['hull_clearance_m'][::2])}m. "
                    + f"Safety clearance min/end action/long {compact(safety)}m. "
                    + ('SAFETY INTRUSION IN ACTION.' if safety[0] < 0 else 'SAFETY INTRUSION IF HELD.' if safety[2] < 0 else 'Safety clear.')
                )
            state['candidate_schema'] = {
                'tracking': 'Region error at forecast_times_s is distance to the rear interior set, zero inside. Main forecast holds observed target turn; the straightening alternative holds current target direction/speed. Both are hypothetical; prefer progress that survives either. Interior leaves quarter radial width as margin, with equal physical margin at angular edges. Official region is unchanged.',
                'clearance': 'Hull and safety clearance cover BOTH the moving target and every nearby obstacle. Target safety radius=max(minimum following distance,combined hulls); obstacle safety uses its avoidance circle. Hull arrays=[action min,long min]; safety=[action min,end,long min,end]. Safety<0=intrusion; hull<=0=contact. Minima include shared start; endpoints allow recovery. Ordinary obstacles frozen at current positions.',
                'bypass': 'Bypass rows=forecast_times_s, columns=[clockwise,counterclockwise] around the target in its predicted rotating frame. Tangent+arc remaining distances to the rear interior; unsafe starts include radial escape. References combine the CURRENT initial relative path direction (distance/12s) with target velocity. Geometric aids, not executable commands or obstacle-aware paths. Compare with current bypass_remaining_m; zero means inside. All 128 choices remain available.',
            }
            if state['task']['mode'] != 'recover':
                state['candidate_schema'].pop('bypass')
                state['candidate_schema']['tracking'] = 'Point error at forecast_times_s is distance to the current-direction rear point minus tolerance, floored at zero. Reference deviations command/actual first are to target velocity plus gradual point-error correction, not heading alignment or an executable command. All 128 choices remain available.'
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
        if observation['task']['code'] == 'SpecialTargetTrackMode':
            for name, facts in criteria.items():
                unit, speed_key = ('knots','command_speed_knots') if 'command_speed_knots' in facts else ('m/s','command_speed_mps')
                suffix = 'knots' if unit=='knots' else 'mps'
                compact = lambda v: json.dumps(rounded(v,1),allow_nan=False,separators=(',',':'))
                statement = f"Set {facts[speed_key]:.3f} {unit}, absolute course {facts['course_deg']:g} deg. "
                if 'setup_error_m' in facts:
                    statement += (f"Setup point error after action/if held {compact(facts['setup_error_m'])} m. "
                        f"Setup reference deviation command/actual end {compact(facts['setup_guidance_error_deg'])} deg. ")
                    if facts['capture_time_s'] is not None:
                        statement += (f"First capture at {facts['capture_time_s']:.2f}s: speed error {facts['capture_speed_error_'+suffix]:.3f} {unit}, "
                                      f"pointing error {facts['capture_pointing_error_deg']:.1f} deg. ")
                elif 'end_position_error_m' in facts:
                    statement += (f"Action-end errors: position {facts['end_position_error_m']:.1f} m, bearing {facts['end_bearing_error_deg']:.1f} deg, "
                        f"speed {facts['end_speed_error_'+suffix]:.3f} {unit}, pointing {facts['end_pointing_error_deg']:.1f} deg. "
                        f"Reference deviation command/actual end {compact(facts['guidance_error_deg'])} deg. "
                        + ('First capture' if facts['pass_is_capture'] else 'Closest pass without capture')
                        + f" at {facts['pass_time_s']:.2f}s: position error {facts['pass_position_error_m']:.1f} m, "
                        f"bearing error {facts['pass_bearing_error_deg']:.1f} deg, speed error {facts['pass_speed_error_'+suffix]:.3f} {unit}, "
                        f"pointing error {facts['pass_pointing_error_deg']:.1f} deg. ")
                else:
                    statement += f"End speed {facts['end_speed_'+suffix]:.3f} {unit}, displacement {facts['travel_m']:.1f} m. "
                if 'clearance_m' in facts:
                    statement += f"Hull clearance {compact(facts['clearance_m'])} m."
                criteria[name] = statement
        for name, facts in criteria.items():
            if isinstance(facts, dict):
                criteria[name] = json.dumps(rounded(facts), ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        return payload
    return build
