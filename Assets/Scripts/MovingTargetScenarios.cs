using System;
using System.Collections.Generic;
using UnityEngine;

// Shared target motion schedules; task-specific judgement stays with each task.
public static class MovingTargetScenarios
{
    public static readonly string[] Names = { "Straight", "Gentle turn", "Speed / stop", "Irregular" };
    public static FollowingScenarioData Build(BoatDynamics2D boat, int index, float InitialReferenceCourse,
        float TargetSpeedKnots, float MaxSpeedKnots, float AlongMin, float AlongMax,
        float AcrossMin, float AcrossMax, float DurationSeconds, bool earlyManeuvers = false)
    {
        if (index < 0 || index >= Names.Length) throw new FormatException("Invalid accompany scenario.");
        if (!BoatDynamics2D.IsFinite(DurationSeconds) || DurationSeconds <= 0) throw new FormatException("Invalid experiment duration.");
        var f = FollowingTarget.Direction(InitialReferenceCourse); var right = new Vector2(f.y, -f.x);
        // Keep the default start independent of the requested offset. Moving both
        // the band and the boat made edits look ineffective in the boat-follow camera.
        float along = -240, across = 170;
        // Still start outside the band if the user moves it over the default start.
        if (along >= AlongMin && along <= AlongMax && across >= AcrossMin && across <= AcrossMax)
            along = AlongMin - 80;
        var initial = f * along + right * across;
        // A sustained turn must exercise more than a complete revolution after acquisition.
        float duration = index == 1 ? Mathf.Max(DurationSeconds, 360)
            : index == 3 ? Mathf.Max(DurationSeconds, 240) : DurationSeconds;
        float targetSpeed = TargetSpeedKnots;
        float targetAcceleration = 1.46f, targetDeceleration = 1.75f;
        var segments = new List<FollowingScenarioData.Segment> {
            new() { start_s = 0, speed_knots = targetSpeed, yaw_rate_deg_s = 0 }
        };
        if (index == 1) segments.Add(new() { start_s = earlyManeuvers ? 6 : Mathf.Min(48, duration * .4f), speed_knots = TargetSpeedKnots, yaw_rate_deg_s = 2 });
        if (index == 2)
        {
            segments.Add(new() { start_s = earlyManeuvers ? 6 : DurationSeconds * .35f, speed_knots = TargetSpeedKnots * .6f });
            segments.Add(new() { start_s = earlyManeuvers ? 12 : DurationSeconds * .6f, speed_knots = 0 });
            segments.Add(new() { start_s = earlyManeuvers ? 20 : DurationSeconds * .8f, speed_knots = TargetSpeedKnots });
        }
        if (index == 3)
        {
            float ownLimit = boat.maxForwardSpeed;
            if (MaxSpeedKnots > 0) ownLimit = Mathf.Min(ownLimit, MaxSpeedKnots * DiscreteAction.MetresPerSecondPerKnot);
            targetSpeed = Mathf.Min(TargetSpeedKnots, 8, ownLimit * .45f / DiscreteAction.MetresPerSecondPerKnot);
            targetAcceleration = Mathf.Min(.1f, boat.maxAcceleration * .15f);
            targetDeceleration = Mathf.Min(.1f, boat.maxDeceleration * .15f);
            float maxAlong = Mathf.Max(Mathf.Abs(AlongMin), Mathf.Abs(AlongMax));
            float maxAcross = Mathf.Max(Mathf.Abs(AcrossMin), Mathf.Abs(AcrossMax));
            float radiusSquared = maxAlong * maxAlong + maxAcross * maxAcross;
            float radius = Mathf.Sqrt(radiusSquared);
            float u = targetSpeed * DiscreteAction.MetresPerSecondPerKnot;
            // Bound the speed of every offset point in either turn direction, not just the target speed.
            // |v_offset|^2 <= u^2 + 2*u*|yaw|*maxAcross + yaw^2*radiusSquared.
            float budget = ownLimit * .9f;
            float yawBudget = (Mathf.Sqrt(u*u*maxAcross*maxAcross + radiusSquared*(budget*budget-u*u))
                - u*maxAcross) / radiusSquared * Mathf.Rad2Deg;
            float maximumYaw = Mathf.Min(2, boat.maxYawRateDeg * .25f, yawBudget);
            float yawAcceleration = Mathf.Min(.14f, boat.maxAngularAccelerationDeg * .1f,
                .35f * Mathf.Min(boat.maxAcceleration, boat.maxDeceleration) / radius * Mathf.Rad2Deg);
            segments = BuildIrregularSegments(duration, targetSpeed, maximumYaw, yawAcceleration, earlyManeuvers ? 6 : 40);
        }
        return new FollowingScenarioData {
            scenario_id = "accompany_" + index, duration_s = duration,
            initial_state = new TargetRouteData.InitialState { x = initial.x, y = initial.y,
                heading_deg = 90 - InitialReferenceCourse, speed = 0, yaw_rate_deg = 0 },
            target = new FollowingScenarioData.Target { x = 0, y = 0, radius_m = 6,
                course_deg = InitialReferenceCourse, speed_knots = targetSpeed,
                acceleration_mps2 = targetAcceleration, deceleration_mps2 = targetDeceleration },
            segments = segments.ToArray(), obstacles = Array.Empty<FollowingScenarioData.Obstacle>()
        };
    }

    private static List<FollowingScenarioData.Segment> BuildIrregularSegments(
        float duration, float maximumSpeed, float maximumYaw, float yawAcceleration, float initialStraightSeconds)
    {
        // Repeatable irregular manoeuvres, private to Unity; never exposed as future model input.
        var random = new System.Random(20261028);
        var segments = new List<FollowingScenarioData.Segment>();
        float elapsed = 0, yaw = 0;
        int turnSign = -1;
        void Add(float span, float speed, float rate)
        {
            if (span <= 0 || elapsed >= duration) return;
            float next = Mathf.Min(duration, elapsed + span);
            if (next <= elapsed) return;
            segments.Add(new() { start_s = elapsed, speed_knots = speed, yaw_rate_deg_s = rate });
            elapsed = next;
        }
        Add(Mathf.Min(initialStraightSeconds, duration), maximumSpeed, 0);
        while (elapsed < duration)
        {
            if (random.NextDouble() < .8) turnSign = -turnSign;
            float desiredYaw = turnSign * maximumYaw * (.55f + .45f * (float)random.NextDouble());
            float speed = maximumSpeed * (.75f + .25f * (float)random.NextDouble());
            float ramp = Mathf.Abs(desiredYaw-yaw) / yawAcceleration;
            float rampElapsed = 0;
            while (rampElapsed < ramp && elapsed < duration)
            {
                float step = Mathf.Min(.5f, ramp-rampElapsed, duration-elapsed);
                if (step <= 0) break;
                Add(step, speed, Mathf.Lerp(yaw, desiredYaw, (rampElapsed + step*.5f)/ramp));
                rampElapsed += step;
            }
            yaw = desiredYaw;
            Add(8 + 10*(float)random.NextDouble(), speed, yaw);
        }
        return segments;
    }

}
