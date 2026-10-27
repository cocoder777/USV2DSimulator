using System;
using Newtonsoft.Json;
using UnityEngine;

// Simulation-only schedule. Never transmitted to the decision client.
[Serializable]
public sealed class FollowingScenarioData
{
    [JsonProperty(Required = Required.Always)] public string scenario_id;
    [JsonProperty(Required = Required.Always)] public TargetRouteData.InitialState initial_state;
    [JsonProperty(Required = Required.Always)] public float duration_s;
    [JsonProperty(Required = Required.Always)] public Target target;
    [JsonProperty(Required = Required.Always)] public Segment[] segments;
    public Obstacle[] obstacles = Array.Empty<Obstacle>();
    [JsonIgnore] public float? PlannedTargetSafetyClearance { get; private set; }

    [Serializable] public sealed class Target
    {
        [JsonProperty(Required = Required.Always)] public float x, y, course_deg, speed_knots, radius_m;
        public float acceleration_mps2 = 1, deceleration_mps2 = 1;
    }
    [Serializable] public sealed class Segment
    {
        [JsonProperty(Required = Required.Always)] public float start_s, speed_knots, yaw_rate_deg_s;
    }
    [Serializable] public sealed class Obstacle
    {
        public float x, y, radius_m = 3, avoidance_radius_m = 50;
        public float amplitude_m, frequency_rad_s = 0.04f, axis_course_deg = 90;
    }
    public static FollowingScenarioData Parse(string json)
    {
        var data = JsonConvert.DeserializeObject<FollowingScenarioData>(json);
        if (data == null || string.IsNullOrWhiteSpace(data.scenario_id) || data.initial_state == null
            || data.target == null || data.segments == null || data.segments.Length == 0
            || !Positive(data.duration_s) || !Positive(data.target.radius_m)
            || !Finite(data.target.x, data.target.y, data.target.course_deg, data.target.speed_knots,
                data.initial_state.x, data.initial_state.y, data.initial_state.heading_deg,
                data.initial_state.speed, data.initial_state.yaw_rate_deg)
            || data.target.speed_knots < 0 || data.target.speed_knots > 200
            || !Positive(data.target.acceleration_mps2) || !Positive(data.target.deceleration_mps2)
            || data.target.course_deg < 0 || data.target.course_deg >= 360)
            throw new FormatException("Invalid following scenario or target state.");
        float previous = -1;
        foreach (var segment in data.segments)
        {
            if (segment == null || !Finite(segment.start_s, segment.speed_knots, segment.yaw_rate_deg_s)
                || segment.start_s <= previous || segment.start_s < 0 || segment.start_s >= data.duration_s
                || segment.speed_knots < 0 || segment.speed_knots > 200 || Mathf.Abs(segment.yaw_rate_deg_s) > 12)
                throw new FormatException("Invalid target motion segment.");
            previous = segment.start_s;
        }
        if (data.segments[0].start_s != 0) throw new FormatException("Target schedule must start at zero.");
        if (data.obstacles == null) throw new FormatException("Invalid following obstacles.");
        foreach (var o in data.obstacles)
            if (o == null || !Finite(o.x, o.y, o.radius_m, o.avoidance_radius_m, o.amplitude_m,
                    o.frequency_rad_s, o.axis_course_deg) || !Positive(o.radius_m)
                || o.avoidance_radius_m < o.radius_m || o.amplitude_m < 0 || o.frequency_rad_s < 0)
                throw new FormatException("Invalid following obstacle.");
        data.ValidateTargetClearance();
        return data;
    }

    // Same movement for validation and live simulation; target scripts never enter model requests.
    public void AdvanceTarget(ref Vector2 position, ref float speed, ref float course, ref double elapsed, float dt)
    {
        double end = elapsed + dt;
        while (elapsed < end - 1e-9)
        {
            int index = 0;
            while (index + 1 < segments.Length && segments[index + 1].start_s <= elapsed + 1e-7) index++;
            var segment = segments[index];
            double boundary = index + 1 < segments.Length ? segments[index + 1].start_s : end;
            float span = (float)(Math.Min(end, boundary) - elapsed);
            float desired = segment.speed_knots * DiscreteAction.MetresPerSecondPerKnot;
            float acceleration = desired >= speed ? target.acceleration_mps2 : target.deceleration_mps2;
            float nextSpeed = Mathf.MoveTowards(speed, desired, acceleration * span);
            position += FollowingTarget.Direction(course + segment.yaw_rate_deg_s * span / 2) * ((speed + nextSpeed) / 2 * span);
            course = BoatDynamics2D.NormalizeDegrees(course + segment.yaw_rate_deg_s * span);
            speed = nextSpeed; elapsed += span;
        }
    }

    public void ValidateTargetClearance()
    {
        PlannedTargetSafetyClearance = null;
        if (obstacles.Length == 0) return;
        var position = new Vector2(target.x, target.y);
        float speed = target.speed_knots * DiscreteAction.MetresPerSecondPerKnot, course = target.course_deg;
        double elapsed = 0;
        do
        {
            var previous = position;
            AdvanceTarget(ref position, ref speed, ref course, ref elapsed, (float)Math.Min(.02, duration_s - elapsed));
            foreach (var obstacle in obstacles)
            {
                var center = new Vector2(obstacle.x, obstacle.y);
                // Bounding the whole patrol by a disk makes this independent of obstacle phase.
                float clearance = DynamicObstacleField.SweptDistance(previous - center, position - center)
                    - target.radius_m - obstacle.avoidance_radius_m - obstacle.amplitude_m;
                PlannedTargetSafetyClearance = Mathf.Min(PlannedTargetSafetyClearance ?? float.PositiveInfinity, clearance);
                if (clearance <= 0) throw new FormatException("Target route overlaps an obstacle safety/patrol envelope. Adjust the scenario layout.");
            }
        } while (elapsed < duration_s - 1e-6);
    }
    private static bool Positive(float v) => BoatDynamics2D.IsFinite(v) && v > 0;
    private static bool Finite(params float[] values) => Array.TrueForAll(values, BoatDynamics2D.IsFinite);
}
