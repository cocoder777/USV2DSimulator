"""Generate the shared reference curve and synchronize the algorithm handoff copy."""
import argparse
import copy
import json
import math
from pathlib import Path

STEP_X_METERS = 0.1
INTERPOLATION = "piecewise_quintic_smoothstep"


def evaluate_segment(start, end, t):
    """Return y, dy/dx, d2y/dx2, d3y/dx3 for 0 <= t <= 1."""
    width = end["x"] - start["x"]
    height = end["y"] - start["y"]
    return (
        start["y"] + height * (10 * t**3 - 15 * t**4 + 6 * t**5),
        height / width * 30 * t**2 * (1 - t)**2,
        height / width**2 * (60 * t - 180 * t**2 + 120 * t**3),
        height / width**3 * (60 - 360 * t + 360 * t**2),
    )


def build_curve(route, step=STEP_X_METERS):
    if not math.isfinite(step) or step <= 0:
        raise ValueError("X sampling step must be finite and positive")
    points = route["waypoints"]
    if len(points) < 2:
        raise ValueError("At least two task points are required")
    ids = set()
    for index, point in enumerate(points):
        expected = "start" if index == 0 else "end" if index == len(points) - 1 else "checkpoint"
        if not isinstance(point.get("id"), str) or not point["id"].strip() or point["id"] in ids:
            raise ValueError("Task point IDs must be non-empty and unique")
        ids.add(point["id"])
        if point.get("role") != expected:
            raise ValueError("Expected start, checkpoints and end in order")
        if any(type(point.get(axis)) not in (int, float) or not math.isfinite(point[axis]) for axis in ("x", "y")):
            raise ValueError("Task point coordinates must be finite numbers")
        if index and point["x"] <= points[index - 1]["x"]:
            raise ValueError("Quintic y(x) requires strictly increasing task point X")
    samples = [{"x": points[0]["x"], "y": points[0]["y"]}]
    for start, end in zip(points, points[1:]):
        width = end["x"] - start["x"]
        count = math.ceil(round(width / step, 10))
        for index in range(1, count + 1):
            if index == count:
                samples.append({"x": end["x"], "y": end["y"]})
            else:
                x = start["x"] + index * step
                y = evaluate_segment(start, end, (x - start["x"]) / width)[0]
                samples.append({"x": round(x, 10), "y": round(y, 10)})
    result = copy.deepcopy(route)
    result["schema_version"] = 2
    result["route_id"] = "usv_fixed_curve_001"
    result["path"] = {
        "type": "sampled_curve",
        "interpolation": INTERPOLATION,
        "sampling_step_x_m": step,
        "samples": samples,
    }
    return result


def sync_interface(info, route):
    result = copy.deepcopy(info)
    result["document_version"] = max(result.get("document_version", 1), 2)
    result["route"] = copy.deepcopy(route)
    definitions = result["route_field_definitions"]
    definitions["schema_version"]["description"] = "路线文件版本，当前为 2；Unity 同时兼容版本 1 的任务点折线。与 UDP 协议版本独立。"
    definitions["waypoints"]["description"] = "有序任务点（起点、必过点、终点），只定义通过要求；版本 2 的参考路线以 path.samples 为准，不能直连任务点代替曲线。"
    for name, kind, unit, description in [
        ("path", "object", None, "版本 2 必填的采样曲线定义；Unity 和算法共用同一组 samples。"),
        ("path.type", "string", None, "固定为 sampled_curve。"),
        ("path.interpolation", "string", None, "piecewise_quintic_smoothstep：各任务点间使用五次平滑 y(x)，节点切线与曲率连续。"),
        ("path.sampling_step_x_m", "number", "m", "沿 X 轴的采样步长（不是等弧长），当前为 0.1；每段末尾保留精确任务点。"),
        ("path.samples", "array<object>", None, f"按 X 严格递增排列的参考曲线采样点，当前共 {len(route['path']['samples'])} 个。算法应跟踪该数组，任务点仍独立用于通过判断。"),
        ("path.samples[].x", "number", "m", "参考曲线采样点的世界 X 坐标。"),
        ("path.samples[].y", "number", "m", "参考曲线采样点的世界 Y 坐标。"),
    ]:
        definitions[name] = {"type": kind, "unit": unit, "description": description, "access": "file_route"}
    result["route_task_rules"]["reference_geometry"] = "以 route.path.samples 为参考曲线，依次经过 waypoints 中的五个任务点；不再直连任务点生成折线。相邻采样点是平滑曲线的细分近似。"
    result["route_generation"] = {
        "script": "TestData/generate_fixed_curve.py",
        "input": "TestData/fixed_route.json 的 waypoints；initial_state、巡航速度和容差保留。",
        "formula": "t=(x-x0)/(x1-x0); y=y0+(y1-y0)*(10*t^3-15*t^4+6*t^5)",
        "continuity": "每个任务点处 dy/dx 和 d2y/dx2 都为 0；切线沿 +X，曲率为 0，整体 C2 连续。水平段保持直线。",
        "constraint": "只支持任务点 X 严格递增的本固定测试任务，不是通用任意二维样条。",
        "regenerate": "python TestData/generate_fixed_curve.py",
        "check": "python TestData/generate_fixed_curve.py --check",
        "notes": "由生成器统一更新路线源和本资料副本。路线格式为 2 不改变现有 UDP v2；采样点是文件数据，不通过 UDP 发送。",
    }
    return result


def serialized(value):
    return json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Verify both files without writing")
    args = parser.parse_args()
    directory = Path(__file__).resolve().parent
    route_file, info_file = directory / "fixed_route.json", directory / "algorithm_interface.json"
    try:
        current = json.loads(route_file.read_text(encoding="utf-8"))
        info = json.loads(info_file.read_text(encoding="utf-8"))
        route = build_curve(current)
        updated = sync_interface(info, route)
        if args.check:
            if current != route or info != updated:
                parser.exit(1, "Curve data or algorithm interface copy is stale; regenerate it.\n")
        else:
            # Both outputs are fully prepared and validated before writing either file.
            route_text, info_text = serialized(route), serialized(updated)
            route_file.write_text(route_text, encoding="utf-8")
            info_file.write_text(info_text, encoding="utf-8")
        print(f"{'Verified' if args.check else 'Generated'} {len(route['path']['samples'])} shared curve samples; task points preserved.")
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f"Curve generation failed: {error}\n")


if __name__ == "__main__":
    main()
