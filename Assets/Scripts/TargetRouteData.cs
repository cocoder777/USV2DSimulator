using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;
using System.Text;

// Required members distinguish a missing coordinate from a legitimate zero.
[DataContract]
public sealed class TargetRouteData
{
    [DataMember(IsRequired = true)] public int schema_version;
    [DataMember(IsRequired = true)] public string route_id;
    [DataMember(IsRequired = true)] public RouteCoordinates coordinate_system;
    [DataMember(IsRequired = true)] public RoutePoint[] waypoints;
    [DataMember] public CurvePath path;
    [DataMember] public InitialState initial_state;
    [DataMember] public float arrival_radius_m;
    [DataMember] public float stop_speed_mps;

    [DataContract]
    public sealed class InitialState
    {
        [DataMember(IsRequired = true)] public float x;
        [DataMember(IsRequired = true)] public float y;
        [DataMember(IsRequired = true)] public float heading_deg;
        [DataMember(IsRequired = true)] public float speed;
        [DataMember(IsRequired = true)] public float yaw_rate_deg;
    }

    [DataContract]
    public sealed class CurvePath
    {
        [DataMember(IsRequired = true)] public string type;
        [DataMember(IsRequired = true)] public string interpolation;
        [DataMember] public float sampling_step_x_m;
        [DataMember] public float sampling_step_m;
        [DataMember] public int[] waypoint_sample_indices;
        [DataMember(IsRequired = true)] public RouteSample[] samples;
    }

    [DataContract]
    public sealed class RouteSample
    {
        [DataMember(IsRequired = true)] public float x;
        [DataMember(IsRequired = true)] public float y;
    }

    // v1 is a task-point polyline; v2 is y(x); v3 is an ordered parametric 2D curve.
    public RouteSample[] GetReferenceSamples()
    {
        if (schema_version >= 2) return path.samples;
        var samples = new RouteSample[waypoints.Length];
        for (int i = 0; i < samples.Length; i++)
            samples[i] = new RouteSample { x = waypoints[i].x, y = waypoints[i].y };
        return samples;
    }

    [DataContract]
    public sealed class RouteCoordinates
    {
        [DataMember(IsRequired = true)] public string frame;
        [DataMember(IsRequired = true)] public string position_unit;
        [DataMember(IsRequired = true)] public float unity_units_per_meter;
        [DataMember(IsRequired = true)] public string heading_unit;
        [DataMember(IsRequired = true)] public string heading_zero_axis;
        [DataMember(IsRequired = true)] public string positive_rotation;
    }

    [Serializable, DataContract]
    public sealed class RoutePoint
    {
        [DataMember(IsRequired = true)] public string id;
        [DataMember(IsRequired = true)] public string role;
        [DataMember(IsRequired = true)] public float x;
        [DataMember(IsRequired = true)] public float y;
    }

    public static TargetRouteData Parse(string json)
    {
        using (var stream = new MemoryStream(Encoding.UTF8.GetBytes(json)))
        {
            var serializer = new DataContractJsonSerializer(typeof(TargetRouteData));
            var route = serializer.ReadObject(stream) as TargetRouteData;
            if (route == null || route.schema_version < 1 || route.schema_version > 3
                || string.IsNullOrWhiteSpace(route.route_id))
                throw new FormatException("Expected schema_version 1..3 and a non-empty route_id.");
            var coordinates = route.coordinate_system;
            if (coordinates == null || coordinates.frame != "unity_world_xy"
                || coordinates.position_unit != "m" || coordinates.unity_units_per_meter != 1f
                || coordinates.heading_unit != "deg" || coordinates.heading_zero_axis != "+X"
                || coordinates.positive_rotation != "counterclockwise")
                throw new FormatException("Expected Unity world XY, metres, and counterclockwise degrees from +X.");
            if (route.waypoints == null || route.waypoints.Length < 2)
                throw new FormatException("At least two waypoints are required.");
            var ids = new HashSet<string>(StringComparer.Ordinal);
            for (int i = 0; i < route.waypoints.Length; i++)
            {
                var point = route.waypoints[i];
                string expected = i == 0 ? "start" : i == route.waypoints.Length - 1 ? "end" : "checkpoint";
                if (point == null || string.IsNullOrWhiteSpace(point.id) || !ids.Add(point.id))
                    throw new FormatException("Waypoint IDs must be non-empty and unique.");
                if (point.role != expected)
                    throw new FormatException("Expected start, intermediate checkpoints, and end in array order.");
                if (float.IsNaN(point.x) || float.IsInfinity(point.x)
                    || float.IsNaN(point.y) || float.IsInfinity(point.y))
                    throw new FormatException("Waypoint coordinates must be finite.");
            }
            if (route.schema_version == 1)
            {
                if (route.initial_state == null) route.initial_state = new InitialState {
                    x = route.waypoints[0].x, y = route.waypoints[0].y };
                if (route.arrival_radius_m == 0) route.arrival_radius_m = 1;
                if (route.stop_speed_mps == 0) route.stop_speed_mps = 0.1f;
            }
            else
            {
                var document = Newtonsoft.Json.Linq.JObject.Parse(json);
                foreach (string field in new[] { "initial_state", "arrival_radius_m", "stop_speed_mps" })
                    if (document[field] == null || document[field].Type == Newtonsoft.Json.Linq.JTokenType.Null)
                        throw new FormatException("Missing task field: " + field);
                route.ValidateCurve();
            }
            route.ValidateTask();
            return route;
        }
    }

    private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);

    private void ValidateTask()
    {
        if (initial_state == null || !Finite(initial_state.x) || !Finite(initial_state.y)
            || !Finite(initial_state.heading_deg) || !Finite(initial_state.speed) || !Finite(initial_state.yaw_rate_deg)
            || !Finite(arrival_radius_m) || arrival_radius_m <= 0
            || !Finite(stop_speed_mps) || stop_speed_mps < 0)
            throw new FormatException("Task initial state must be finite; radius positive and stop speed nonnegative.");
        double dx = initial_state.x - waypoints[0].x, dy = initial_state.y - waypoints[0].y;
        if (dx * dx + dy * dy > (double)arrival_radius_m * arrival_radius_m)
            throw new FormatException("Initial position must be inside the start arrival region.");
    }

    private void ValidateCurve()
    {
        if (schema_version == 3) { ValidateParametricCurve(); return; }
        const float tolerance = 0.0001f;
        if (path == null || path.type != "sampled_curve"
            || path.interpolation != "piecewise_quintic_smoothstep"
            || !Finite(path.sampling_step_x_m) || path.sampling_step_x_m <= 0
            || path.samples == null || path.samples.Length < 2)
            throw new FormatException("v2 requires a sampled quintic curve, positive X step and at least two samples.");
        for (int i = 0; i < path.samples.Length; i++)
        {
            var sample = path.samples[i];
            if (sample == null || !Finite(sample.x) || !Finite(sample.y))
                throw new FormatException("Curve sample coordinates must be present and finite.");
            if (i > 0 && (sample.x <= path.samples[i - 1].x
                || sample.x - path.samples[i - 1].x > path.sampling_step_x_m + tolerance))
                throw new FormatException("Curve samples must have increasing X and respect the declared X step.");
        }
        int sampleIndex = 0;
        for (int i = 0; i < waypoints.Length; i++)
        {
            var point = waypoints[i];
            if (i > 0 && point.x <= waypoints[i - 1].x)
                throw new FormatException("Quintic y(x) task points require strictly increasing X.");
            while (sampleIndex < path.samples.Length && path.samples[sampleIndex].x < point.x - tolerance)
                sampleIndex++;
            if (sampleIndex == path.samples.Length
                || Math.Abs(path.samples[sampleIndex].x - point.x) > tolerance
                || Math.Abs(path.samples[sampleIndex].y - point.y) > tolerance
                || (i == 0 && sampleIndex != 0)
                || (i == waypoints.Length - 1 && sampleIndex != path.samples.Length - 1))
                throw new FormatException("Curve must start/end at the task endpoints and include each task point in order.");
        }
    }

    private void ValidateParametricCurve()
    {
        const double tolerance = 0.0001;
        if (path == null || path.type != "sampled_curve" || path.interpolation != "parametric_cubic_hermite"
            || !Finite(path.sampling_step_m) || path.sampling_step_m <= 0
            || path.samples == null || path.samples.Length < 2
            || path.waypoint_sample_indices == null || path.waypoint_sample_indices.Length != waypoints.Length)
            throw new FormatException("v3 requires a sampled 2D curve, positive distance step and waypoint sample indices.");
        for (int i = 0; i < path.samples.Length; i++)
        {
            var sample = path.samples[i];
            if (sample == null || !Finite(sample.x) || !Finite(sample.y))
                throw new FormatException("Curve samples must be finite.");
            if (i == 0) continue;
            double dx = sample.x - path.samples[i - 1].x, dy = sample.y - path.samples[i - 1].y;
            double distance = Math.Sqrt(dx * dx + dy * dy);
            if (distance <= 0 || distance > path.sampling_step_m + tolerance)
                throw new FormatException("2D curve samples must be distinct and respect the distance step.");
        }
        for (int i = 0; i < waypoints.Length; i++)
        {
            int index = path.waypoint_sample_indices[i];
            if (index < 0 || index >= path.samples.Length
                || (i > 0 && index <= path.waypoint_sample_indices[i - 1])
                || (i == 0 && index != 0) || (i == waypoints.Length - 1 && index != path.samples.Length - 1)
                || Math.Abs(path.samples[index].x - waypoints[i].x) > tolerance
                || Math.Abs(path.samples[index].y - waypoints[i].y) > tolerance)
                throw new FormatException("2D curve must include all task points in route order and start/end at its endpoints.");
        }
    }
}
