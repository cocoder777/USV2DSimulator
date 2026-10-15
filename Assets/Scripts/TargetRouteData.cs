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

    [DataContract]
    public sealed class CurvePath
    {
        [DataMember(IsRequired = true)] public string type;
        [DataMember(IsRequired = true)] public string interpolation;
        [DataMember(IsRequired = true)] public float sampling_step_x_m;
        [DataMember(IsRequired = true)] public RouteSample[] samples;
    }

    [DataContract]
    public sealed class RouteSample
    {
        [DataMember(IsRequired = true)] public float x;
        [DataMember(IsRequired = true)] public float y;
    }

    // v1 uses the task points as a polyline; v2 uses the explicit reference curve.
    public RouteSample[] GetReferenceSamples()
    {
        if (schema_version == 2) return path.samples;
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

    [DataContract]
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
            if (route == null || (route.schema_version != 1 && route.schema_version != 2)
                || string.IsNullOrWhiteSpace(route.route_id))
                throw new FormatException("Expected schema_version 1 or 2 and a non-empty route_id.");
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
            if (route.schema_version == 2) route.ValidateCurve();
            return route;
        }
    }

    private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);

    private void ValidateCurve()
    {
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
}
