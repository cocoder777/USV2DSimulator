using System;
using Newtonsoft.Json.Linq;

public static class ObservationBuilder
{
    // Synthetic small-area geographic frame: Unity +X east, +Y north, 1 unit = 1 metre.
    public const double OriginLon = 120.5, OriginLat = 30.2, EarthRadius = 6378137;
    private static JArray LonLat(double x, double y) => new JArray(
        Math.Round(OriginLon + x / (EarthRadius * Math.Cos(OriginLat * Math.PI / 180)) * 180 / Math.PI, 8),
        Math.Round(OriginLat + y / EarthRadius * 180 / Math.PI, 8));

    public static JObject ForModel(BoatDynamics2D boat, MissionManager mission, float actionDuration = 0.4f)
    {
        if (mission?.Route == null || mission.Progress.Next == null)
            throw new InvalidOperationException("No active task.");
        var route = mission.Route;
        var samples = new JArray();
        foreach (var p in route.GetReferenceSamples()) samples.Add(LonLat(p.x, p.y));
        var waypoints = new JArray();
        foreach (var p in route.waypoints) waypoints.Add(LonLat(p.x, p.y));
        var speeds = new JArray();
        for (int i = 0; i < 8; i++) speeds.Add(DiscreteAction.Speed(i, boat));
        return new JObject {
            ["planning"] = new JObject { ["action_duration_s"] = actionDuration,
                ["simulation_paused_while_deciding"] = true },
            ["action_space"] = new JObject { ["speeds_mps"] = speeds,
                ["heading_offsets_deg"] = new JArray(DiscreteAction.HeadingOffsetsDeg) },
            ["boat"] = new JObject { ["lon_lat"] = LonLat(boat.Position.x, boat.Position.y),
                ["heading_deg"] = boat.HeadingDeg },
            ["task"] = new JObject {
                ["planRouteNum"] = mission.MapLevel, ["route_id"] = route.route_id,
                ["currentRoute"] = Math.Max(0, mission.Progress.NextIndex - 1),
                ["next_waypoint_index"] = mission.Progress.NextIndex,
                ["planWaypointNum"] = route.waypoints.Length, ["planWaypointList"] = waypoints,
                ["reference_lon_lat"] = samples, ["origin_lon_lat"] = new JArray(OriginLon, OriginLat),
                ["arrival_radius_m"] = route.arrival_radius_m, ["stop_speed_mps"] = route.stop_speed_mps }
        };
    }
}
