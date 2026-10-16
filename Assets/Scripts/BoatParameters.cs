using System;

[Serializable]
public sealed class BoatParameters
{
    public float max_forward_speed, max_reverse_speed, max_acceleration;
    public float max_angular_acceleration_deg, max_yaw_rate_deg;
    public float acceleration_time_constant, angular_acceleration_time_constant;
    public float linear_damping, angular_damping, fixed_dt;

    public static BoatParameters Capture(BoatDynamics2D boat) => new BoatParameters {
        max_forward_speed = boat.maxForwardSpeed, max_reverse_speed = boat.maxReverseSpeed,
        max_acceleration = boat.maxAcceleration, max_angular_acceleration_deg = boat.maxAngularAccelerationDeg,
        max_yaw_rate_deg = boat.maxYawRateDeg, acceleration_time_constant = boat.accelerationTimeConstant,
        angular_acceleration_time_constant = boat.angularAccelerationTimeConstant,
        linear_damping = boat.linearDamping, angular_damping = boat.angularDamping,
        fixed_dt = UnityEngine.Time.fixedDeltaTime
    };
}

// Shared numerical plant for actual execution and single-axis input forecasts.
public static class BoatScalarDynamics
{
    public static (float value, float actuator) Step(float value, float actuator, float command,
        float timeConstant, float damping, float dt, float minimum, float maximum)
    {
        float blend = timeConstant <= 0 ? 1 : 1 - UnityEngine.Mathf.Exp(-dt / timeConstant);
        actuator += (command - actuator) * blend;
        value = UnityEngine.Mathf.Clamp(value + (actuator - damping * value) * dt, minimum, maximum);
        return (value, actuator);
    }
    public static float Predict(float value, float actuator, float command, float timeConstant,
        float damping, float dt, int steps, float minimum, float maximum)
    {
        for (int i = 0; i < steps; i++)
            (value, actuator) = Step(value, actuator, command, timeConstant, damping, dt, minimum, maximum);
        return value;
    }
}
