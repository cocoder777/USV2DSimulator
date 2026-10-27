using System;
using Newtonsoft.Json.Linq;

public static class ObservationBuilder
{
    public const double OriginLon = 120.5, OriginLat = 30.2, EarthRadius = 6378137;
    public const float ImmediateStopSpeedMps = 0.000001f;

    public static JObject ForStop(BoatDynamics2D boat, MissionManager mission, float actionDuration)
    {
        var observation = ForModel(boat, mission, actionDuration, true);
        observation["unity_only"]["stop_speed_knots"] = ImmediateStopSpeedMps / DiscreteAction.MetresPerSecondPerKnot;
        return observation;
    }
    public static JArray LonLat(double x, double y) => new JArray(
        Math.Round(OriginLon + x / (EarthRadius * Math.Cos(OriginLat * Math.PI / 180)) * 180 / Math.PI, 8),
        Math.Round(OriginLat + y / EarthRadius * 180 / Math.PI, 8));

    public static JObject ForModel(BoatDynamics2D boat, MissionManager mission, float actionDuration = BoatParameters.DefaultActionDuration, bool allowCompleted = false)
    {
        if (mission == null || !mission.HasTask || (mission.Completed && !allowCompleted))
            throw new InvalidOperationException("No active task.");
        var route = mission.Route;
        var position = LonLat(boat.Position.x, boat.Position.y);
        var waypoints = new JArray();
        // The map's first point is an initial-position marker, not a protocol target.
        for (int i = 1; route != null && i < route.waypoints.Length; i++)
        {
            var p = route.waypoints[i];
            var coordinates = LonLat(p.x, p.y);
            waypoints.Add(new JObject { ["routeLon"] = coordinates[0].DeepClone(),
                ["routeLat"] = coordinates[1].DeepClone(), ["routeSpeed"] = p.routeSpeed });
        }
        // Velocity direction is not bow direction. At rest no course is available.
        float course = BoatDynamics2D.NormalizeDegrees(90f - boat.HeadingDeg + (boat.Speed < 0 ? 180 : 0));
        const int ownPlatform = 1001;
        var situation = new JArray { Record(ownPlatform, position,
            boat.Speed / DiscreteAction.MetresPerSecondPerKnot,
            Math.Abs(boat.Speed) < 0.000001f ? (double?)null : course, 2) };
        var radii = new JObject { [ownPlatform.ToString()] = boat.CollisionRadius };
        if (mission.IsFollowing)
        {
            situation.Add(mission.Following.Observe());
            radii[FollowingTarget.PlatformId.ToString()] = mission.Following.Scenario.target.radius_m;
        }
        foreach (JObject obstacle in boat.GetComponent<DynamicObstacleField>()?.Observe() ?? new JArray())
        {
            situation.Add(obstacle.DeepClone());
            radii[((int)obstacle["platformId"]).ToString()] = boat.GetComponent<DynamicObstacleField>().CollisionRadius((int)obstacle["platformId"]);
        }
        foreach (JObject point in waypoints) point["routeSpeed"] = (double)point["routeSpeed"] / DiscreteAction.MetresPerSecondPerKnot;
        var observation = new JObject {
            ["params"] = mission.IsFollowing ? new JObject { ["platCode"] = ownPlatform,
                ["targetPlatformId"] = FollowingTarget.PlatformId, ["minFollowingDistanceM"] = mission.Following.MinDistance,
                ["maxFollowingDistanceM"] = mission.Following.MaxDistance } : new JObject {
                ["platCode"] = ownPlatform, ["planRouteNum"] = mission.MapLevel,
                ["currentRoute"] = Math.Max(0, mission.Progress.NextIndex - 1),
                ["planWaypointNum"] = waypoints.Count, ["planWaypointList"] = waypoints },
            ["situation"] = situation,
            ["unity_only"] = new JObject {
                ["stop_speed_knots"] = (route?.stop_speed_mps ?? 0.1f) / DiscreteAction.MetresPerSecondPerKnot,
                ["collision_radii_m"] = radii,
                ["action_duration_s"] = actionDuration,
                ["max_speed_knots"] = boat.maxForwardSpeed / DiscreteAction.MetresPerSecondPerKnot,
                ["execution"] = new JObject {
                    ["motion_model"] = "bounded_acceleration",
                    ["max_acceleration_mps2"] = boat.maxAcceleration,
                    ["max_deceleration_mps2"] = boat.maxDeceleration,
                    ["max_yaw_rate_deg_s"] = boat.maxYawRateDeg,
                    ["max_angular_acceleration_deg_s2"] = boat.maxAngularAccelerationDeg,
                    ["heading_course_deg"] = BoatDynamics2D.NormalizeDegrees(90f - boat.HeadingDeg),
                    ["yaw_rate_deg_s"] = -boat.YawRateDeg,
                    ["active_target_course_deg"] = boat.NavigationTargetCourseDeg,
                    ["active_course_error_deg"] = boat.NavigationCourseErrorDeg,
                    ["fixed_dt_s"] = UnityEngine.Time.fixedDeltaTime,
                    ["hull_length_m"] = boat.lengthMetres, ["hull_beam_m"] = boat.beamMetres,
                    ["obstacle_avoidance_radii_m"] = boat.GetComponent<DynamicObstacleField>()?.AvoidanceRadii() ?? new JObject() } }
        };
        if (!mission.IsFollowing) observation["unity_only"]["arrival_radius_m"] = route.arrival_radius_m;
        return observation;
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
