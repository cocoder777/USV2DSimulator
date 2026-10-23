using System;
using Newtonsoft.Json.Linq;

public static class ObservationBuilder
{
    public const double OriginLon = 120.5, OriginLat = 30.2, EarthRadius = 6378137;
    public const float ImmediateStopSpeedMps = 0.000001f;

    public static JObject ForStop(BoatDynamics2D boat, MissionManager mission, float actionDuration)
    {
        var observation = ForModel(boat, mission, actionDuration);
        observation["unity_only"]["stop_speed_knots"] = ImmediateStopSpeedMps / DiscreteAction.MetresPerSecondPerKnot;
        return observation;
    }
    public static JArray LonLat(double x, double y) => new JArray(
        Math.Round(OriginLon + x / (EarthRadius * Math.Cos(OriginLat * Math.PI / 180)) * 180 / Math.PI, 8),
        Math.Round(OriginLat + y / EarthRadius * 180 / Math.PI, 8));

    public static JObject ForModel(BoatDynamics2D boat, MissionManager mission, float actionDuration = 0.4f)
    {
        if (mission?.Route == null || mission.Progress.Next == null)
            throw new InvalidOperationException("No active task.");
        var route = mission.Route;
        var position = LonLat(boat.Position.x, boat.Position.y);
        var waypoints = new JArray();
        foreach (var p in route.waypoints)
        {
            var coordinates = LonLat(p.x, p.y);
            waypoints.Add(new JObject { ["routeLon"] = coordinates[0].DeepClone(),
                ["routeLat"] = coordinates[1].DeepClone(), ["routeSpeed"] = p.routeSpeed });
        }
        var speeds = new JArray();
        for (int i = 0; i < 8; i++) speeds.Add(DiscreteAction.Speed(i, boat) / DiscreteAction.MetresPerSecondPerKnot);
        // Velocity direction is not bow direction. At rest no course is available.
        float course = UnityEngine.Mathf.Repeat(90f - boat.HeadingDeg + (boat.Speed < 0 ? 180 : 0), 360);
        const int ownPlatform = 1001;
        var situation = new JArray { Record(ownPlatform, position,
            boat.Speed / DiscreteAction.MetresPerSecondPerKnot,
            Math.Abs(boat.Speed) < 0.000001f ? (double?)null : course, 2) };
        var radii = new JObject { [ownPlatform.ToString()] = DynamicObstacleField.BoatRadius };
        foreach (JObject obstacle in boat.GetComponent<DynamicObstacleField>()?.Observe() ?? new JArray())
        {
            situation.Add(obstacle.DeepClone());
            radii[((int)obstacle["platformId"]).ToString()] = DynamicObstacleField.ObstacleRadius;
        }
        foreach (JObject point in waypoints) point["routeSpeed"] = (double)point["routeSpeed"] / DiscreteAction.MetresPerSecondPerKnot;
        return new JObject {
            ["params"] = new JObject {
                ["platCode"] = ownPlatform, ["planRouteNum"] = mission.MapLevel,
                ["currentRoute"] = Math.Max(0, mission.Progress.NextIndex - 1),
                ["planWaypointNum"] = route.waypoints.Length, ["planWaypointList"] = waypoints },
            ["situation"] = situation,
            ["unity_only"] = new JObject {
                ["next_waypoint_index"] = mission.Progress.NextIndex,
                ["arrival_radius_m"] = route.arrival_radius_m,
                ["stop_speed_knots"] = route.stop_speed_mps / DiscreteAction.MetresPerSecondPerKnot,
                ["collision_radii_m"] = radii,
                ["planning"] = new JObject { ["action_duration_s"] = actionDuration,
                    ["simulation_paused_while_deciding"] = true },
                ["action_space"] = new JObject { ["speeds_knots"] = speeds,
                    ["courses_deg"] = new JArray(DiscreteAction.CoursesDeg) },
                ["execution"] = new JObject { ["speed_transition_time_s"] = boat.speedResponseTime,
                    ["course_transition_time_s"] = boat.headingResponseTime } }
        };
    }

    public static JObject Record(int platformId, JArray position, double speed, double? course, int sign = 0)
        => new JObject {
            ["platformId"] = platformId, ["longitude"] = position[0].DeepClone(),
            ["latitude"] = position[1].DeepClone(), ["targetSpeed"] = speed,
            ["targetCourse"] = course.HasValue ? new JValue(course.Value) : JValue.CreateNull(),
            ["direction"] = 0, ["sign"] = sign, ["height"] = 0, ["type"] = 0,
            ["targetTypeConfid"] = JValue.CreateNull(), ["threatLevel"] = JValue.CreateNull(),
            ["batch"] = platformId };
}
