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
    public static JObject ForModel(BoatDynamics2D boat, MissionManager mission, int actionCount = 1, float actionDuration = 0.4f)
    {
        if (mission?.Route == null) throw new InvalidOperationException("No task loaded.");
        var target = mission.Progress.Next;
        if (target == null) throw new InvalidOperationException("No current target.");
        var parameters = JObject.FromObject(BoatParameters.Capture(boat));
        parameters.Remove("fixed_dt");
        return new JObject {
            ["coordinates"] = "Global XY in metres. Heading in degrees: 0=east, 90=north; counterclockwise positive.",
            ["planning"] = new JObject { ["action_count"] = actionCount, ["action_duration_s"] = actionDuration,
                ["simulation_paused_while_deciding"] = true },
            ["actions"] = new JObject {
                ["acceleration_fractions"] = new JArray(DiscreteActionPlan.AccelerationFractions),
                ["turn_fractions"] = new JArray(DiscreteActionPlan.TurnFractions) },
            ["boat"] = new JObject { ["position_xy_m"] = new JArray(boat.Position.x, boat.Position.y),
                ["heading_deg"] = boat.HeadingDeg, ["speed_mps"] = boat.Speed, ["yaw_rate_deg_s"] = boat.YawRateDeg,
                ["actuator_acceleration_mps2"] = boat.ActualAccelerationValue,
                ["actuator_angular_acceleration_deg_s2"] = boat.ActualAngularAccelerationDeg },
            ["target"] = new JObject { ["position_xy_m"] = new JArray(target.x, target.y),
                ["arrival_radius_m"] = mission.Route.arrival_radius_m,
                ["requires_stop"] = mission.Progress.NextIndex == mission.Route.waypoints.Length - 1,
                ["stop_speed_mps"] = mission.Route.stop_speed_mps },
            ["dynamics"] = parameters,
            ["motion"] = "Signed speed moves along heading. Commands drive actuators with first-order lag. "
                + "Speed derivative = actuator acceleration - linear damping * speed; "
                + "yaw-rate derivative = actuator angular acceleration - angular damping * yaw rate. "
                + "Speeds are clamped to their limits. Zero time constant means no actuator lag."
        };
    }
}
