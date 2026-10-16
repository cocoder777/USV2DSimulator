using System;

[Serializable]
public sealed class DiscreteBoatAction
{
    public int acceleration_index;
    public int turn_index;
}

// Wire contract shared with Python. No HTTP or model code.
public static class DiscreteActionPlan
{
    public static readonly float[] AccelerationFractions = { -1f, -2f / 3, -1f / 3, 0, 0.25f, 0.5f, 0.75f, 1f };
    public static readonly float[] TurnFractions = { -1f, -0.5f, -0.25f, 0, 0.25f, 0.5f, 1f };
    public static void Validate(DiscreteBoatAction[] actions)
    {
        if (actions == null || actions.Length < 1 || actions.Length > 10)
            throw new FormatException("Expected 1..10 paired actions.");
        foreach (var action in actions)
            if (action == null || action.acceleration_index < 0 || action.acceleration_index >= 8
                || action.turn_index < 0 || action.turn_index >= 7)
                throw new FormatException("Expected acceleration_index 0..7 and turn_index 0..6.");
    }
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
