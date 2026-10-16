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

    public MissionProgress(TargetRouteData route) { Route = route ?? throw new ArgumentNullException(nameof(route)); }

    public void Reset(Vector2 position, float speed)
    {
        NextIndex = 0;
        Evaluate(position, position, speed);
    }

    public void Evaluate(Vector2 previous, Vector2 current, float speed)
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
