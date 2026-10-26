using System;
using UnityEngine;

// One set of editable defaults; internal physics always uses metres and seconds.
[Serializable]
public sealed class BoatParameters
{
    public float max_forward_speed, max_reverse_speed, max_acceleration, max_deceleration;
    public float max_angular_acceleration_deg, max_yaw_rate_deg, length_m, beam_m, arrival_radius_multiplier, fixed_dt;
    public const float DefaultMaxSpeedKnots = 50f;
    public const float DefaultWaypointRadiusMultiplier = 5f;
    public const float DefaultAcceleration = 1.46f, DefaultDeceleration = 1.75f;
    public const float DefaultYawRate = 12f, DefaultAngularAcceleration = 4f;
    public const float DefaultLength = 11.6f, DefaultBeam = 3.4f, DefaultActionDuration = 1f;

    public static BoatParameters Capture(BoatDynamics2D boat) => new BoatParameters {
        max_forward_speed = boat.maxForwardSpeed, max_reverse_speed = boat.maxReverseSpeed,
        max_acceleration = boat.maxAcceleration, max_deceleration = boat.maxDeceleration,
        max_angular_acceleration_deg = boat.maxAngularAccelerationDeg, max_yaw_rate_deg = boat.maxYawRateDeg,
        length_m = boat.lengthMetres, beam_m = boat.beamMetres, arrival_radius_multiplier = boat.waypointRadiusMultiplier, fixed_dt = Time.fixedDeltaTime
    };
    public static void Restore(BoatDynamics2D boat)
    {
        boat.maxForwardSpeed = DefaultMaxSpeedKnots * DiscreteAction.MetresPerSecondPerKnot;
        boat.maxReverseSpeed = 1f;
        boat.maxAcceleration = DefaultAcceleration; boat.maxDeceleration = DefaultDeceleration;
        boat.maxYawRateDeg = DefaultYawRate; boat.maxAngularAccelerationDeg = DefaultAngularAcceleration;
        boat.lengthMetres = DefaultLength; boat.beamMetres = DefaultBeam;
        boat.waypointRadiusMultiplier = DefaultWaypointRadiusMultiplier;
    }
}

// The Python predictor implements these same bounded kinematics, including turn braking.
public static class BoatMotion
{
    public static float Speed(float initial, float target, float acceleration, float deceleration, float t)
        => Mathf.MoveTowards(initial, target, (target >= initial ? acceleration : deceleration) * t);

    // Minimum-time bounded angular trajectory ending at delta with zero yaw.
    // Reconstructing its remaining phases each physical step preserves the held trajectory.
    public static (float angle, float yaw) Turn(float delta, float initialYaw, float maxYaw, float acceleration, float t)
    {
        if (t <= 0) return (0, initialYaw);
        if (Mathf.Abs(delta) < 1e-7f && Mathf.Abs(initialYaw) < 1e-7f) return (delta, 0);
        float switching = delta - initialYaw * Mathf.Abs(initialYaw) / (2 * acceleration);
        float direction = switching == 0 ? (initialYaw == 0 ? 1 : -Mathf.Sign(initialYaw)) : Mathf.Sign(switching);
        float velocity = direction * initialYaw, distance = direction * delta;
        float peak = Mathf.Sqrt(Mathf.Max(0, acceleration * distance + velocity * velocity / 2));
        float top = Mathf.Min(maxYaw, peak);
        float accelerating = Mathf.Max(0, (top - velocity) / acceleration);
        float coasting = peak > maxYaw
            ? Mathf.Max(0, (distance - (2 * top * top - velocity * velocity) / (2 * acceleration)) / top) : 0;
        float braking = top / acceleration;
        if (t >= accelerating + coasting + braking) return (delta, 0);
        float angle = 0;
        float span = Mathf.Min(t, accelerating);
        angle += velocity * span + acceleration * span * span / 2;
        velocity += acceleration * span; t -= span;
        span = Mathf.Min(t, coasting); angle += velocity * span; t -= span;
        span = Mathf.Min(t, braking);
        angle += velocity * span - acceleration * span * span / 2;
        velocity -= acceleration * span;
        return (direction * angle, direction * velocity);
    }
}
