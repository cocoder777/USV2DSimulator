using System;
using Newtonsoft.Json.Linq;

public static class ObservationBuilder
{
    public const double OriginLon = 120.5, OriginLat = 30.2, EarthRadius = 6378137;
    private static JArray LonLat(double x, double y) => new JArray(
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
        return new JObject {
            ["planning"] = new JObject { ["action_duration_s"] = actionDuration,
                ["simulation_paused_while_deciding"] = true },
            ["action_space"] = new JObject { ["speeds_knots"] = speeds,
                ["courses_deg"] = new JArray(DiscreteAction.CoursesDeg) },
            ["boat"] = new JObject { ["longitude"] = position[0].DeepClone(),
                ["latitude"] = position[1].DeepClone(),
                ["targetCourse"] = Math.Abs(boat.Speed) < 0.000001f ? JValue.CreateNull() : new JValue(course),
                ["targetSpeed"] = boat.Speed / DiscreteAction.MetresPerSecondPerKnot },
            // Known simulator configuration, not extra measured ship state.
            ["execution"] = new JObject { ["speed_transition_time_s"] = boat.speedResponseTime,
                ["course_transition_time_s"] = boat.headingResponseTime },
            ["task"] = new JObject {
                ["planRouteNum"] = mission.MapLevel,
                ["currentRoute"] = Math.Max(0, mission.Progress.NextIndex - 1),
                ["next_waypoint_index"] = mission.Progress.NextIndex,
                ["planWaypointNum"] = route.waypoints.Length, ["planWaypointList"] = waypoints,
                ["routeSpeed_unit"] = "m/s",
                ["arrival_radius_m"] = route.arrival_radius_m, ["stop_speed_mps"] = route.stop_speed_mps }
        };
    }
}
