using System;

[Serializable]
public sealed class DiscreteBoatAction
{
    public int speed_index;
    public int heading_index;
}

// Wire contract shared with Python. No HTTP or model code.
public static class DiscreteAction
{
    public static readonly float[] SpeedFractions = { -1f, -0.5f, 0, 0.1f, 0.25f, 0.5f, 0.75f, 1f };
    public static readonly float[] HeadingOffsetsDeg = { 0, 45, 90, 135, 180, -135, -90, -45 };
    public static void Validate(DiscreteBoatAction action)
    {
        if (action == null || action.speed_index < 0 || action.speed_index >= 8
            || action.heading_index < 0 || action.heading_index >= 8)
            throw new FormatException("Expected speed_index and heading_index in 0..7.");
    }
    public static float Speed(int index, BoatDynamics2D boat) => SpeedFractions[index]
        * (SpeedFractions[index] < 0 ? boat.maxReverseSpeed : boat.maxForwardSpeed);
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
