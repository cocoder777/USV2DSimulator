using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;

// Uses the same simulation clock as the boat, including pauses between decisions.
[DisallowMultipleComponent]
public sealed class DynamicObstacleField : MonoBehaviour
{
    public const float ObstacleRadius = 1f;
    // A conservative circular hull for this simplified decision experiment.
    public const float BoatRadius = 0.5f;
    private sealed class Obstacle
    {
        public string id;
        public Vector2 center, axis, position;
        public float phase;
        public LineRenderer circle;
    }
    private readonly List<Obstacle> obstacles = new List<Obstacle>();
    private MissionManager mission;
    private BoatDynamics2D boat;
    private Material material;
    private double elapsed;

    void Awake()
    {
        mission = GetComponent<MissionManager>();
        boat = GetComponent<BoatDynamics2D>();
        material = new Material(Shader.Find("Sprites/Default")) { color = Color.red };
    }
    void OnEnable() { boat.ResetPerformed += ResetField; if (mission.Route != null) ResetField(); }
    void OnDisable() { boat.ResetPerformed -= ResetField; }
    void OnDestroy() { if (material != null) Destroy(material); }

    public void ResetField()
    {
        foreach (var obstacle in obstacles) if (obstacle.circle != null) Destroy(obstacle.circle.gameObject);
        obstacles.Clear(); elapsed = 0;
        var route = mission.Route;
        if (route == null) return;
        for (int i = 1; i < route.waypoints.Length && obstacles.Count < 3; i++)
        {
            var a = route.waypoints[i - 1]; var b = route.waypoints[i];
            Vector2 start = new Vector2(a.x, a.y), end = new Vector2(b.x, b.y);
            Vector2 center = (start + end) * 0.5f;
            var direction = (end - start).normalized;
            var axis = new Vector2(-direction.y, direction.x);
            // Keep the complete patrol outside every arrival circle and the initial hull.
            float clearance = route.arrival_radius_m + ObstacleRadius + BoatRadius + 0.5f;
            bool clear = !Touches(center - axis * 1.5f - boat.Position,
                center + axis * 1.5f - boat.Position, clearance);
            foreach (var p in route.waypoints)
                clear &= !Touches(center - axis * 1.5f - new Vector2(p.x, p.y),
                    center + axis * 1.5f - new Vector2(p.x, p.y), clearance);
            if (!clear) continue;
            var go = new GameObject("MovingObstacle_" + i);
            go.transform.SetParent(transform, false);
            var circle = go.AddComponent<LineRenderer>();
            circle.sharedMaterial = material; circle.useWorldSpace = true;
            circle.startColor = circle.endColor = Color.white;
            circle.startWidth = circle.endWidth = 0.10f;
            circle.positionCount = 48; circle.loop = true; circle.sortingOrder = 3;
            obstacles.Add(new Obstacle { id = "obstacle_" + i, center = center,
                axis = axis, phase = 0,
                position = center, circle = circle });
        }
        Draw();
    }

    private Vector2 PositionAt(Obstacle obstacle, double time) => obstacle.center
        + obstacle.axis * (1.5f * Mathf.Sin((float)(time * 0.4) + obstacle.phase));

    public string Step(float dt, Vector2 previousBoat, Vector2 currentBoat)
    {
        string collision = null;
        foreach (var obstacle in obstacles)
        {
            Vector2 previous = obstacle.position;
            obstacle.position = PositionAt(obstacle, elapsed + dt);
            // Sweep relative motion so crossing a circle between two samples is still a collision.
            if (Touches(previousBoat - previous, currentBoat - obstacle.position,
                ObstacleRadius + BoatRadius)) collision ??= obstacle.id;
        }
        elapsed += dt; Draw(); return collision;
    }

    public static bool Touches(Vector2 start, Vector2 end, float radius)
    {
        Vector2 delta = end - start;
        float t = delta.sqrMagnitude > 0 ? Mathf.Clamp01(-Vector2.Dot(start, delta) / delta.sqrMagnitude) : 0;
        return (start + delta * t).sqrMagnitude <= radius * radius;
    }

    private void Draw()
    {
        foreach (var obstacle in obstacles)
            for (int i = 0; i < obstacle.circle.positionCount; i++)
            {
                float angle = i * 2 * Mathf.PI / obstacle.circle.positionCount;
                obstacle.circle.SetPosition(i, new Vector3(obstacle.position.x + Mathf.Cos(angle) * ObstacleRadius,
                    obstacle.position.y + Mathf.Sin(angle) * ObstacleRadius, -0.02f));
            }
    }

    public JArray Observe()
    {
        var result = new JArray();
        foreach (var obstacle in obstacles)
        {
            var position = ObservationBuilder.LonLat(obstacle.position.x, obstacle.position.y);
            // Derivative of the existing sinusoidal patrol; does not change its motion.
            Vector2 velocity = obstacle.axis * (0.6f * Mathf.Cos((float)(elapsed * 0.4) + obstacle.phase));
            double speed = velocity.magnitude / DiscreteAction.MetresPerSecondPerKnot;
            double? course = speed < 0.000001 ? null : Mathf.Repeat(
                Mathf.Atan2(velocity.x, velocity.y) * Mathf.Rad2Deg, 360);
            var record = ObservationBuilder.Record(2001 + obstacles.IndexOf(obstacle), position, speed, course);
            Vector2 delta = obstacle.position - boat.Position;
            record["direction"] = Mathf.RoundToInt(Mathf.Repeat(Mathf.Atan2(delta.x, delta.y) * Mathf.Rad2Deg, 360)) % 360;
            result.Add(record);
        }
        return result;
    }

    public JArray Snapshot()
    {
        var result = new JArray();
        foreach (var obstacle in obstacles)
            result.Add(new JObject { ["id"] = obstacle.id, ["x"] = obstacle.position.x,
                ["y"] = obstacle.position.y, ["radius_m"] = ObstacleRadius });
        return result;
    }
}
