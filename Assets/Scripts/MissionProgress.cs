using System;
using UnityEngine;

// The only checkpoint/finish judge. Independent of rendering, networking and model answers.
public sealed class MissionProgress
{
    public TargetRouteData Route { get; }
    public int NextIndex { get; private set; }
    public bool Completed => NextIndex == Route.waypoints.Length;
    public TargetRouteData.RoutePoint Next => Completed ? null : Route.waypoints[NextIndex];
    public event Action<int> PointPassed;
    public float?[] PassingMaxSpeeds { get; }
    private readonly bool[] passageFinished;

    public MissionProgress(TargetRouteData route)
    {
        Route = route ?? throw new ArgumentNullException(nameof(route));
        PassingMaxSpeeds = new float?[route.waypoints.Length];
        passageFinished = new bool[route.waypoints.Length];
    }

    public void Reset(Vector2 position, float speed)
    {
        NextIndex = 0;
        Array.Clear(PassingMaxSpeeds, 0, PassingMaxSpeeds.Length);
        Array.Clear(passageFinished, 0, passageFinished.Length);
        Evaluate(position, position, speed);
        // Start is an initial-position marker, not a speed-limited departure gate.
        passageFinished[0] = true;
    }

    public void Evaluate(Vector2 previous, Vector2 current, float speed, float? previousSpeed = null)
    {
        EvaluateProgress(previous, current, speed);
        // Observe the valid target and already-passed circles until first exit.
        // Include swept intersections, even when both physical-step endpoints are outside.
        for (int i = 0; i <= Math.Min(NextIndex, Route.waypoints.Length - 1); i++)
        {
            if (passageFinished[i]) continue;
            var p = Route.waypoints[i];
            var center = new Vector2(p.x, p.y);
            var delta = current - previous;
            var offset = previous - center;
            float a = delta.sqrMagnitude, low = 0, high = 1;
            if (a < 1e-12f)
            {
                if (offset.sqrMagnitude > Route.arrival_radius_m * Route.arrival_radius_m) continue;
            }
            else
            {
                float b = 2 * Vector2.Dot(offset, delta);
                float c = offset.sqrMagnitude - Route.arrival_radius_m * Route.arrival_radius_m;
                float disc = b*b - 4*a*c;
                if (disc < 0) continue;
                low = Mathf.Max(0, (-b - Mathf.Sqrt(disc))/(2*a));
                high = Mathf.Min(1, (-b + Mathf.Sqrt(disc))/(2*a));
                if (low > high) continue;
            }
            float v0 = previousSpeed ?? speed;
            float peak = Mathf.Max(Mathf.Abs(Mathf.Lerp(v0, speed, low)), Mathf.Abs(Mathf.Lerp(v0, speed, high)));
            PassingMaxSpeeds[i] = Mathf.Max(PassingMaxSpeeds[i] ?? 0, peak);
            if (i < NextIndex && (current-center).sqrMagnitude > Route.arrival_radius_m*Route.arrival_radius_m)
                passageFinished[i] = true;
        }
    }

    private void EvaluateProgress(Vector2 previous, Vector2 current, float speed)
    {
        float earliest = 0;
        while (!Completed)
        {
            var point = Next;
            var center = new Vector2(point.x, point.y);
            if (NextIndex == Route.waypoints.Length - 1)
            {
                // Passing through the end at speed is not stopping at the end.
                if ((current - center).sqrMagnitude > Route.arrival_radius_m * Route.arrival_radius_m
                    || Mathf.Abs(speed) > Route.stop_speed_mps) return;
            }
            else
            {
                float contact = Contact(previous, current, center, Route.arrival_radius_m, earliest);
                if (contact < 0) return;
                earliest = contact;
            }
            int passed = NextIndex++;
            PointPassed?.Invoke(passed);
        }
    }

    private static float Contact(Vector2 from, Vector2 to, Vector2 center, float radius, float minimum)
    {
        Vector2 delta = to - from;
        if ((from + delta * minimum - center).sqrMagnitude <= radius * radius) return minimum;
        float a = delta.sqrMagnitude;
        if (a < 1e-12f) return -1;
        Vector2 offset = from - center;
        float b = 2 * Vector2.Dot(offset, delta), c = offset.sqrMagnitude - radius * radius;
        float discriminant = b * b - 4 * a * c;
        if (discriminant < 0) return -1;
        float enter = (-b - Mathf.Sqrt(discriminant)) / (2 * a);
        return enter >= minimum && enter <= 1 ? enter : -1;
    }
}
