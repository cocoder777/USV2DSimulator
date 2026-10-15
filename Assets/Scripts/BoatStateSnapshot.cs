using System;

[Serializable]
public class BoatStateSnapshot
{
    public string type = "state";
    public int version = 2;
    public long state_sequence;
    public string control_epoch;
    public string run_id;
    public string mode;
    public double simulation_time;
    public float x, y, heading_deg, speed, yaw_rate_deg;
    public float acceleration, angular_acceleration_deg;
    public float actual_acceleration, actual_angular_acceleration_deg;
    public bool control_active;
    public bool command_timed_out;
    public double command_age_seconds;

    public static BoatStateSnapshot Capture(BoatDynamics2D boat)
    {
        return new BoatStateSnapshot {
            run_id = boat.RunId, mode = boat.Mode.ToString(), simulation_time = boat.SimulationTime,
            x = boat.Position.x, y = boat.Position.y, heading_deg = boat.HeadingDeg,
            speed = boat.Speed, yaw_rate_deg = boat.YawRateDeg,
            acceleration = boat.AccelerationCommandValue,
            angular_acceleration_deg = boat.AngularAccelerationCommandDeg,
            actual_acceleration = boat.ActualAccelerationValue,
            actual_angular_acceleration_deg = boat.ActualAngularAccelerationDeg
        };
    }
}
