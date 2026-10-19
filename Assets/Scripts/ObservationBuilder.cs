using System;
using Newtonsoft.Json.Linq;

[Serializable]
public sealed class MissionSnapshot
{
    public string route_id, status;
    public int route_revision, passed_count;
    public TargetRouteData.RoutePoint next_target;
    public float arrival_radius_m, stop_speed_mps;

    public static MissionSnapshot Capture(MissionManager mission)
    {
        if (mission == null || mission.Route == null) return null;
        return new MissionSnapshot { route_id = mission.Route.route_id, route_revision = mission.Revision,
            status = mission.Status, passed_count = mission.Progress.NextIndex,
            next_target = mission.Progress.Next, arrival_radius_m = mission.Route.arrival_radius_m,
            stop_speed_mps = mission.Route.stop_speed_mps };
    }
}

public static class ObservationBuilder
{
    // Only measured state, current target and plant parameters; reference curves are for display/export.
    public static JObject ForModel(BoatDynamics2D boat, MissionManager mission, float actionDuration = 0.4f)
    {
        if (mission?.Route == null) throw new InvalidOperationException("No task loaded.");
        var target = mission.Progress.Next;
        if (target == null) throw new InvalidOperationException("No current target.");
        var parameters = JObject.FromObject(BoatParameters.Capture(boat));
        parameters.Remove("fixed_dt");
        parameters.Remove("acceleration_time_constant");
        parameters.Remove("angular_acceleration_time_constant");
        bool pauseWhileDeciding = boat.GetComponent<SimulationRunner>()?.PauseWhileDeciding ?? true;
        return new JObject {
            ["coordinates"] = "Global XY in metres. Heading in degrees: 0=east, 90=north; counterclockwise positive.",
            ["planning"] = new JObject { ["action_duration_s"] = actionDuration,
                ["simulation_paused_while_deciding"] = pauseWhileDeciding },
            ["action_space"] = new JObject {
                ["acceleration_fractions"] = new JArray(DiscreteAction.AccelerationFractions),
                ["turn_fractions"] = new JArray(DiscreteAction.TurnFractions) },
            ["boat"] = new JObject { ["position_xy_m"] = new JArray(boat.Position.x, boat.Position.y),
                ["heading_deg"] = boat.HeadingDeg, ["speed_mps"] = boat.Speed, ["yaw_rate_deg_s"] = boat.YawRateDeg },
            ["target"] = new JObject { ["position_xy_m"] = new JArray(target.x, target.y),
                ["arrival_radius_m"] = mission.Route.arrival_radius_m,
                ["requires_stop"] = mission.Progress.NextIndex == mission.Route.waypoints.Length - 1,
                ["stop_speed_mps"] = mission.Route.stop_speed_mps },
            ["dynamics"] = parameters,
            ["motion"] = "Acceleration commands take effect immediately when simulation resumes. "
                + "Speed derivative = longitudinal command - linear damping * speed; "
                + "yaw-rate derivative = angular command - angular damping * yaw rate. "
                + "Yaw rate changes heading; signed speed moves the boat along its changing heading, with no independent sideways drift. "
                + "The target is fixed in the world; its boat-relative coordinates change as the boat moves and turns. "
                + "Speeds are clamped to their limits. Zero commands allow damping to slow motion; they do not stop it instantly."
        };
    }
}
