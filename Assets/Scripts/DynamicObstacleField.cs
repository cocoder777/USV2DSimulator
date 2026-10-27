using System;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;

// Obstacles share the boat's simulation clock and pause between decisions.
[DisallowMultipleComponent]
public sealed class DynamicObstacleField : MonoBehaviour
{
    public const float DefaultSmallAvoidanceRadius = 50f, DefaultLargeAvoidanceRadius = 90f;
    [Min(0)] public int maximumObstacles = 3;
    [Min(0.1f)] public float smallCollisionRadius = 3f, largeCollisionRadius = 12f;
    [Min(0.1f)] public float smallAvoidanceRadius = DefaultSmallAvoidanceRadius;
    [Range(80, 100)] public float largeAvoidanceRadius = DefaultLargeAvoidanceRadius;
    [Min(0)] public float patrolAmplitude = 20f;
    [Min(0)] public float patrolAngularFrequency = 0.04f;
    [Serializable] public struct Configuration
    {
        public int count;
        public float small_collision_radius, large_collision_radius, small_avoidance_radius, large_avoidance_radius;
        public float patrol_amplitude, patrol_frequency;
    }
    public Configuration Settings => new Configuration { count = maximumObstacles,
        small_collision_radius = smallCollisionRadius, large_collision_radius = largeCollisionRadius,
        small_avoidance_radius = smallAvoidanceRadius, large_avoidance_radius = largeAvoidanceRadius,
        patrol_amplitude = patrolAmplitude, patrol_frequency = patrolAngularFrequency };
    private sealed class Obstacle
    {
        public string id;
        public int platformId;
        public bool large;
        public FollowingScenarioData.Obstacle configured;
        public Vector2 center, axis, position, previousPosition;
        public LineRenderer hull, safety;
    }
    private readonly List<Obstacle> obstacles = new();
    private MissionManager mission;
    private BoatDynamics2D boat;
    private Material material;
    private Transform visualRoot;
    private double elapsed;
    public float? MinimumPhysicalClearance { get; private set; }
    public float? MinimumAvoidanceClearance { get; private set; }
    public float? MinimumTargetPhysicalClearance { get; private set; }
    public float? MinimumTargetSafetyClearance { get; private set; }

    void Awake()
    {
        mission = GetComponent<MissionManager>(); boat = GetComponent<BoatDynamics2D>();
        material = new Material(Shader.Find("Sprites/Default"));
    }
    void OnEnable() { boat.ResetPerformed += ResetField; if (mission.HasTask) ResetField(); }
    void OnDisable() { boat.ResetPerformed -= ResetField; ClearVisuals(); }
    void OnDestroy()
    {
        if (visualRoot != null) Destroy(visualRoot.gameObject);
        if (material != null) Destroy(material);
    }
    public void RestoreDefaults()
    {
        maximumObstacles = 3; smallCollisionRadius = 3; largeCollisionRadius = 12;
        smallAvoidanceRadius = DefaultSmallAvoidanceRadius; largeAvoidanceRadius = DefaultLargeAvoidanceRadius;
        patrolAmplitude = 20; patrolAngularFrequency = 0.04f;
        ResetField();
    }
    private float Radius(Obstacle obstacle) => obstacle.configured?.radius_m ?? (obstacle.large ? largeCollisionRadius : smallCollisionRadius);
    private float Avoidance(Obstacle obstacle) => obstacle.configured?.avoidance_radius_m ?? Mathf.Max(Radius(obstacle),
        obstacle.large ? largeAvoidanceRadius : smallAvoidanceRadius);
    public float CollisionRadius(int platformId) => Radius(obstacles.Find(o => o.platformId == platformId));
    public JObject AvoidanceRadii()
    {
        var result = new JObject();
        foreach (var obstacle in obstacles) result[obstacle.platformId.ToString()] = Avoidance(obstacle);
        return result;
    }
    private LineRenderer MakeCircle(string name, Color color, float width)
    {
        // World-space obstacles must not inherit the moving/turning boat's transform.
        if (visualRoot == null) visualRoot = new GameObject("ObstacleVisuals").transform;
        var go = new GameObject(name); go.transform.SetParent(visualRoot, false);
        var circle = go.AddComponent<LineRenderer>(); circle.sharedMaterial = material;
        circle.useWorldSpace = true; circle.loop = true; circle.positionCount = 64;
        circle.startColor = circle.endColor = color; circle.startWidth = circle.endWidth = width;
        circle.sortingOrder = 3; return circle;
    }
    public void ResetField()
    {
        ClearVisuals();
        elapsed = 0;
        MinimumPhysicalClearance = MinimumAvoidanceClearance = null;
        MinimumTargetPhysicalClearance = MinimumTargetSafetyClearance = null;
        CreateField();
        if (mission.IsFollowing) MeasureFollowingTarget(mission.Following.Position, mission.Following.Position);
    }
    private void ClearVisuals()
    {
        foreach (var obstacle in obstacles)
        {
            // Destroy is deferred; hide immediately so a reset never renders two sets.
            if (obstacle.hull != null) { obstacle.hull.enabled = false; Destroy(obstacle.hull.gameObject); }
            if (obstacle.safety != null) { obstacle.safety.enabled = false; Destroy(obstacle.safety.gameObject); }
        }
        obstacles.Clear();
    }
    private void CreateField()
    {
        if (mission.IsFollowing)
        {
            if (!mission.Following.ObstaclesEnabled) return;
            foreach (var definition in mission.Following.Scenario.obstacles)
            {
                int id = 2001 + obstacles.Count;
                obstacles.Add(new Obstacle { id = "obstacle_" + id, platformId = id, configured = definition,
                    center = new Vector2(definition.x, definition.y), position = new Vector2(definition.x, definition.y),
                    previousPosition = new Vector2(definition.x, definition.y),
                    axis = FollowingTarget.Direction(definition.axis_course_deg),
                    hull = MakeCircle("ObstacleHull_" + id, Color.red, 0.8f),
                    safety = MakeCircle("ObstacleSafety_" + id, new Color(1, .4f, .15f, .65f), .5f) });
            }
            Draw(); return;
        }
        var route = mission.Route;
        if (route == null) return;
        for (int i = 1; i < route.waypoints.Length && obstacles.Count < maximumObstacles; i++)
        {
            var a = route.waypoints[i - 1]; var b = route.waypoints[i];
            var start = new Vector2(a.x, a.y); var end = new Vector2(b.x, b.y);
            var center = (start + end) / 2;
            var direction = (end - start).normalized; var axis = new Vector2(-direction.y, direction.x);
            bool large = obstacles.Count % 2 == 1;
            float safety = Mathf.Max(large ? largeAvoidanceRadius : smallAvoidanceRadius,
                large ? largeCollisionRadius : smallCollisionRadius);
            float clearance = safety + boat.CollisionRadius + 5;
            bool clear = !Touches(center - axis * patrolAmplitude - boat.Position,
                center + axis * patrolAmplitude - boat.Position, clearance);
            foreach (var point in route.waypoints)
                clear &= !Touches(center - axis * patrolAmplitude - new Vector2(point.x, point.y),
                    center + axis * patrolAmplitude - new Vector2(point.x, point.y), clearance);
            foreach (var other in obstacles)
                clear &= (center - other.center).magnitude > safety + Avoidance(other) + 2 * patrolAmplitude;
            if (!clear) continue;
            int platformId = 2001 + obstacles.Count;
            obstacles.Add(new Obstacle { id = "obstacle_" + platformId, platformId = platformId, large = large,
                center = center, axis = axis, position = center,
                hull = MakeCircle("ObstacleHull_" + platformId, Color.red, 0.8f),
                safety = MakeCircle("ObstacleSafety_" + platformId, new Color(1, 0.4f, 0.15f, 0.65f), 0.5f) });
        }
        Draw();
    }
    private Vector2 PositionAt(Obstacle obstacle, double time) => obstacle.center
        + obstacle.axis * ((obstacle.configured?.amplitude_m ?? patrolAmplitude)
            * Mathf.Sin((float)time * (obstacle.configured?.frequency_rad_s ?? patrolAngularFrequency)));
    public string Step(float dt, Vector2 previousBoat, Vector2 currentBoat)
    {
        string collision = null;
        foreach (var obstacle in obstacles)
        {
            var previous = obstacle.position; obstacle.previousPosition = previous; obstacle.position = PositionAt(obstacle, elapsed + dt);
            float distance = SweptDistance(previousBoat - previous, currentBoat - obstacle.position);
            float physical = distance - Radius(obstacle) - boat.CollisionRadius;
            float avoidance = distance - Avoidance(obstacle) - boat.CollisionRadius;
            MinimumPhysicalClearance = Mathf.Min(MinimumPhysicalClearance ?? float.PositiveInfinity, physical);
            MinimumAvoidanceClearance = Mathf.Min(MinimumAvoidanceClearance ?? float.PositiveInfinity, avoidance);
            if (physical <= 0) collision ??= obstacle.id;
        }
        elapsed += dt; Draw(); return collision;
    }
    public string MeasureFollowingTarget(Vector2 previous, Vector2 current)
    {
        string collision = null;
        foreach (var obstacle in obstacles)
        {
            float distance = SweptDistance(previous - obstacle.previousPosition, current - obstacle.position);
            float physical = distance - mission.Following.Scenario.target.radius_m - Radius(obstacle);
            float safety = distance - mission.Following.Scenario.target.radius_m - Avoidance(obstacle);
            MinimumTargetPhysicalClearance = Mathf.Min(MinimumTargetPhysicalClearance ?? float.PositiveInfinity, physical);
            MinimumTargetSafetyClearance = Mathf.Min(MinimumTargetSafetyClearance ?? float.PositiveInfinity, safety);
            if (physical <= 0) collision ??= "target / " + obstacle.id;
        }
        return collision;
    }
    public static float SweptDistance(Vector2 start, Vector2 end)
    {
        var delta = end - start;
        float t = delta.sqrMagnitude > 0 ? Mathf.Clamp01(-Vector2.Dot(start, delta) / delta.sqrMagnitude) : 0;
        return (start + delta * t).magnitude;
    }
    public static bool Touches(Vector2 start, Vector2 end, float radius) => SweptDistance(start, end) <= radius;
    private void Draw()
    {
        foreach (var obstacle in obstacles)
            foreach (var circle in new[] { obstacle.hull, obstacle.safety })
                for (int i = 0; i < circle.positionCount; i++)
                {
                    float angle = i * 2 * Mathf.PI / circle.positionCount;
                    float radius = circle == obstacle.hull ? Radius(obstacle) : Avoidance(obstacle);
                    circle.SetPosition(i, new Vector3(obstacle.position.x + Mathf.Cos(angle) * radius,
                        obstacle.position.y + Mathf.Sin(angle) * radius, -0.02f));
                }
    }
    public JArray Observe()
    {
        var result = new JArray();
        foreach (var obstacle in obstacles)
        {
            var position = ObservationBuilder.LonLat(obstacle.position.x, obstacle.position.y);
            float amplitude = obstacle.configured?.amplitude_m ?? patrolAmplitude;
            float frequency = obstacle.configured?.frequency_rad_s ?? patrolAngularFrequency;
            var velocity = obstacle.axis * (amplitude * frequency * Mathf.Cos((float)elapsed * frequency));
            double speed = velocity.magnitude / DiscreteAction.MetresPerSecondPerKnot;
            double? course = speed < 0.000001 ? null : Mathf.Repeat(Mathf.Atan2(velocity.x, velocity.y) * Mathf.Rad2Deg, 360);
            var record = ObservationBuilder.Record(obstacle.platformId, position, speed, course);
            var delta = obstacle.position - boat.Position;
            record["direction"] = Mathf.RoundToInt(Mathf.Repeat(Mathf.Atan2(delta.x, delta.y) * Mathf.Rad2Deg, 360)) % 360;
            result.Add(record);
        }
        return result;
    }
    public JArray Snapshot()
    {
        var result = new JArray();
        foreach (var obstacle in obstacles)
            result.Add(new JObject { ["id"] = obstacle.id, ["platform_id"] = obstacle.platformId,
                ["size"] = obstacle.large ? "large" : "small", ["x"] = obstacle.position.x, ["y"] = obstacle.position.y,
                ["radius_m"] = Radius(obstacle), ["avoidance_radius_m"] = Avoidance(obstacle) });
        return result;
    }
}
