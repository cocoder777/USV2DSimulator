using System;

[Serializable]
public class BoatStateSnapshot
{
    public string type = "state";
    public int version = 4;
    public string session_id, episode_id;
    public long tick;
    public long state_sequence;
    public string run_id;
    public string mode;
    public double simulation_time;
    public float x, y, heading_deg, speed, yaw_rate_deg;
    public float acceleration, angular_acceleration_deg;
    public float actual_acceleration, actual_angular_acceleration_deg;
    public bool waiting_for_decision;
    public BoatParameters parameters;
    public MissionSnapshot mission;
    public string scheduler_state, decision_id;
    public int action_count;
    public float decision_period_s, action_duration_s;

    public string ToJson() => Newtonsoft.Json.JsonConvert.SerializeObject(this);

    public static BoatStateSnapshot Capture(BoatDynamics2D boat)
    {
        var runner = boat.GetComponent<SimulationRunner>();
        return new BoatStateSnapshot {
            session_id = boat.SessionId, episode_id = boat.EpisodeId, tick = boat.Tick,
            run_id = boat.RunId, mode = boat.Mode.ToString(), simulation_time = boat.SimulationTime,
            x = boat.Position.x, y = boat.Position.y, heading_deg = boat.HeadingDeg,
            speed = boat.Speed, yaw_rate_deg = boat.YawRateDeg,
            acceleration = boat.AccelerationCommandValue,
            angular_acceleration_deg = boat.AngularAccelerationCommandDeg,
            actual_acceleration = boat.ActualAccelerationValue,
            actual_angular_acceleration_deg = boat.ActualAngularAccelerationDeg,
            parameters = BoatParameters.Capture(boat), mission = MissionSnapshot.Capture(boat.GetComponent<MissionManager>()),
            waiting_for_decision = runner?.IsWaiting ?? false,
            scheduler_state = runner?.State.ToString(), decision_id = runner?.DecisionId,
            action_count = runner?.actionCount ?? 0, decision_period_s = runner?.DecisionPeriod ?? 0, action_duration_s = runner?.actionDuration ?? 0
        };
    }
}
