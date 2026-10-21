using System;

[Serializable]
public sealed class DiscreteBoatAction
{
    public float speed; // knots, nonnegative target ground speed
    public float course; // degrees clockwise from north, target ground course
}

// Wire contract shared with Python. No HTTP or model code.
public static class DiscreteAction
{
    public const float MetresPerSecondPerKnot = 1852f / 3600f;
    public static readonly float[] SpeedFractions = { 0, 0.05f, 0.1f, 0.2f, 0.35f, 0.5f, 0.75f, 1f };
    public static readonly float[] CoursesDeg = { 0, 45, 90, 135, 180, 225, 270, 315 };
    public static void Validate(DiscreteBoatAction action)
    {
        if (action == null || !BoatDynamics2D.IsFinite(action.speed) || action.speed < 0
            || !BoatDynamics2D.IsFinite(action.course) || action.course < 0 || action.course >= 360)
            throw new FormatException("Expected nonnegative speed (knots) and course in [0,360).");
    }
    public static float Speed(int index, BoatDynamics2D boat) => SpeedFractions[index] * boat.maxForwardSpeed;
    public static int Steps(float duration, float dt)
    {
        if (!BoatDynamics2D.IsFinite(duration) || !BoatDynamics2D.IsFinite(dt) || dt <= 0 || duration <= 0)
            throw new ArgumentException("Durations must be finite and positive.");
        float ratio = duration / dt;
        int count = UnityEngine.Mathf.RoundToInt(ratio);
        if (count < 1 || count > 100000 || UnityEngine.Mathf.Abs(count - ratio) > 0.0001f)
            throw new ArgumentException("Duration must contain whole fixed steps.");
        return count;
    }
}
