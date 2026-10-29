using System;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;

// Target motion and physical-step measurements share SimulationRunner's paused clock.
[DisallowMultipleComponent]
public sealed class FollowingTarget : MonoBehaviour
{
    public const int PlatformId = 3001;
    public static readonly string[] ScenarioNames = { "Straight", "Gentle turn", "Complex", "Irregular" };
    public static readonly string[] ScenarioFiles = { "straight", "turn", "complex", "irregular" };
    public FollowingScenarioData Scenario { get; private set; }
    public int ScenarioIndex { get; private set; }
    public float MinDistance { get; private set; } = 100;
    public float MaxDistance { get; private set; } = 200;
    public float MaxSpeedKnots { get; private set; }
    public bool ObstaclesEnabled { get; private set; } = true;
    public Vector2 Position { get; private set; }
    public float Course { get; private set; }
    public float ReferenceCourse { get; private set; }
    public float Speed { get; private set; }
    public double Elapsed { get; private set; }
    public bool Completed => Scenario != null && Elapsed + Time.fixedDeltaTime / 2 >= Scenario.duration_s;
    public float Distance { get; private set; }
    public float AlongBehind { get; private set; }
    public float CrossTrack { get; private set; }
    public float RangeRate { get; private set; }
    public float MinimumDistance { get; private set; }
    public float MinimumClearance { get; private set; }
    public double InBandTime { get; private set; }
    public double FollowingTime { get; private set; }
    public double TooCloseTime { get; private set; }
    public double TooFarTime { get; private set; }
    public double TaskTime { get; private set; }
    public double FrontTime { get; private set; }
    public double? FirstEntryTime { get; private set; }
    public double LongestFollowingTime { get; private set; }
    public double AfterEntryTime { get; private set; }
    public double AfterEntryFollowingTime { get; private set; }
    public float RelativeBearing => BoatDynamics2D.NormalizeDegrees(Mathf.Atan2(CrossTrack, -AlongBehind) * Mathf.Rad2Deg);
    public bool InSector => Distance >= MinDistance && Distance <= MaxDistance && AlongBehind >= 0;
    public string TrackingStatus => Distance < MinDistance ? "Too close" : Distance > MaxDistance ? "Too far"
        : AlongBehind < 0 ? "Ahead of target" : "In rear sector";
    private BoatDynamics2D boat;
    private MissionManager mission;
    private Transform visualRoot;
    private Material material;
    private LineRenderer hull, minimumRing, maximumRing, leftEdge, rightEdge, arrow, reference, trail;
    private bool stationary;
    private double continuousFollowingTime;
    private readonly List<Vector3> history = new();
    public static Vector2 Direction(float course) => new Vector2(Mathf.Sin(course * Mathf.Deg2Rad), Mathf.Cos(course * Mathf.Deg2Rad));
    void Awake() { boat = GetComponent<BoatDynamics2D>(); mission = GetComponent<MissionManager>(); }
    public bool ConfigureDistances(float minimum, float maximum, float maximumSpeedKnots = 0)
    {
        if (!BoatDynamics2D.IsFinite(minimum) || !BoatDynamics2D.IsFinite(maximum)
            || !BoatDynamics2D.IsFinite(maximumSpeedKnots) || maximumSpeedKnots < 0
            || minimum < 0 || maximum < minimum || maximum <= boat.CollisionRadius + (Scenario?.target.radius_m ?? 6)) return false;
        MinDistance = minimum; MaxDistance = maximum; MaxSpeedKnots = maximumSpeedKnots;
        boat.ResetBoat(); return true;
    }
    public void SetObstaclesEnabled(bool enabled)
    {
        if (ObstaclesEnabled == enabled) return;
        ObstaclesEnabled = enabled; boat.ResetBoat();
    }
    public void Load(FollowingScenarioData scenario, int index)
    {
        if (MaxDistance <= boat.CollisionRadius + scenario.target.radius_m)
            throw new FormatException("Following region must extend outside the combined hull radii.");
        Scenario = scenario; ScenarioIndex = index;
    }
    public void ResetTarget()
    {
        ClearVisuals();
        if (!mission.IsFollowing || Scenario == null) return;
        Position = new Vector2(Scenario.target.x, Scenario.target.y);
        Course = Scenario.target.course_deg; Speed = Scenario.target.speed_knots * DiscreteAction.MetresPerSecondPerKnot;
        ReferenceCourse = Course; stationary = Speed < 0.1f;
        Elapsed = InBandTime = FollowingTime = TooCloseTime = TooFarTime = 0;
        TaskTime = FrontTime = LongestFollowingTime = continuousFollowingTime = AfterEntryTime = AfterEntryFollowingTime = 0;
        FirstEntryTime = null;
        MinimumDistance = MinimumClearance = float.PositiveInfinity;
        Measure(boat.Position, boat.Position, Position, 0);
        CreateVisuals(); Draw();
    }
    public string Step(float dt, Vector2 previousBoat, Vector2 currentBoat, bool stopping = false)
    {
        var previousTarget = Position;
        var position = Position; float speed = Speed, course = Course; double elapsed = Elapsed;
        Scenario.AdvanceTarget(ref position, ref speed, ref course, ref elapsed, dt);
        Position = position; Speed = speed; Course = course; Elapsed = elapsed;
        stationary = Speed < (stationary ? 0.2f : 0.1f);
        if (!stationary) ReferenceCourse = Course;
        Measure(previousBoat, currentBoat, previousTarget, dt, !stopping);
        Draw();
        return MinimumClearance <= 0 ? "following_target_3001" : null;
    }
    private void Measure(Vector2 previousBoat, Vector2 currentBoat, Vector2 previousTarget, float dt, bool countTask = true)
    {
        var delta = Position - currentBoat; Distance = delta.magnitude;
        var forward = Direction(ReferenceCourse); var right = new Vector2(forward.y, -forward.x);
        AlongBehind = Vector2.Dot(delta, forward); CrossTrack = Vector2.Dot(currentBoat - Position, right);
        var velocity = new Vector2(Mathf.Cos(boat.HeadingDeg * Mathf.Deg2Rad), Mathf.Sin(boat.HeadingDeg * Mathf.Deg2Rad)) * boat.Speed;
        RangeRate = Distance > 1e-5 ? Vector2.Dot(delta, Direction(Course) * Speed - velocity) / Distance : 0;
        float swept = DynamicObstacleField.SweptDistance(previousTarget - previousBoat, delta);
        MinimumDistance = Mathf.Min(MinimumDistance, swept);
        MinimumClearance = Mathf.Min(MinimumClearance, swept - boat.CollisionRadius - Scenario.target.radius_m);
        if (!countTask) return;
        bool enteredPreviously = FirstEntryTime.HasValue;
        TaskTime += dt;
        // Duration fractions for the linearly interpolated relative physical-step segment.
        double insideMin = InsideFraction(previousTarget - previousBoat, delta, MinDistance, false);
        double insideMax = InsideFraction(previousTarget - previousBoat, delta, MaxDistance);
        TooCloseTime += dt * insideMin; TooFarTime += dt * (1 - insideMax);
        InBandTime += dt * (insideMax - insideMin);
        if (AlongBehind < 0) FrontTime += dt;
        if (InSector)
        {
            FollowingTime += dt; continuousFollowingTime += dt;
            LongestFollowingTime = Math.Max(LongestFollowingTime, continuousFollowingTime);
            if (!FirstEntryTime.HasValue) FirstEntryTime = TaskTime;
        }
        else continuousFollowingTime = 0;
        if (enteredPreviously)
        { AfterEntryTime += dt; if (InSector) AfterEntryFollowingTime += dt; }
    }
    private static double InsideFraction(Vector2 a, Vector2 b, float radius, bool inclusive = true)
    {
        var v = b - a; double aa = v.sqrMagnitude, bb = 2 * Vector2.Dot(a, v), cc = a.sqrMagnitude - radius * radius;
        if (aa < 1e-12) return (inclusive ? cc <= 0 : cc < 0) ? 1 : 0;
        double disc = bb * bb - 4 * aa * cc;
        if (disc <= 0) return 0;
        double lo = Math.Max(0, (-bb - Math.Sqrt(disc)) / (2 * aa)), hi = Math.Min(1, (-bb + Math.Sqrt(disc)) / (2 * aa));
        return Math.Max(0, hi - lo);
    }
    public JObject Observe()
    {
        var record = ObservationBuilder.Record(PlatformId, ObservationBuilder.LonLat(Position.x, Position.y),
            Speed / DiscreteAction.MetresPerSecondPerKnot, Speed < 1e-6 ? (double?)null : Course);
        var d = Position - boat.Position;
        record["direction"] = Mathf.RoundToInt(Mathf.Repeat(Mathf.Atan2(d.x, d.y) * Mathf.Rad2Deg, 360)) % 360;
        return record;
    }
    public JObject Parameters() => new JObject { ["platCode"] = 1001, ["targetPlatCode"] = PlatformId,
        ["followDistanceMinM"] = MinDistance, ["followDistanceMaxM"] = MaxDistance, ["maxSpeed"] = MaxSpeedKnots };
    public JObject StatusParams() => new JObject { ["targetPlatCode"] = PlatformId,
        ["relativeDistanceM"] = Distance, ["relativeBearingDeg"] = RelativeBearing,
        ["inRange"] = Distance >= MinDistance && Distance <= MaxDistance ? 1 : 0,
        ["distanceOutM"] = Distance - Mathf.Clamp(Distance, MinDistance, MaxDistance),
        ["targetSpeed"] = Speed / DiscreteAction.MetresPerSecondPerKnot, ["targetCourse"] = ReferenceCourse };
    public JObject Snapshot() => new JObject { ["platform_id"] = PlatformId, ["x"] = Position.x, ["y"] = Position.y,
        ["speed_mps"] = Speed, ["course_deg"] = Course, ["radius_m"] = Scenario.target.radius_m,
        ["distance_m"] = Distance, ["range_rate_mps"] = RangeRate, ["along_behind_m"] = AlongBehind,
        ["cross_track_m"] = CrossTrack, ["reference_course_deg"] = ReferenceCourse,
        ["relative_bearing_deg"] = RelativeBearing, ["in_sector"] = InSector, ["status"] = TrackingStatus };
    public JObject Metrics() => new JObject { ["min_distance_m"] = MinDistance, ["max_distance_m"] = MaxDistance,
        ["guidance"] = "rear_sector_following", ["sector_angle_deg"] = 180, ["max_speed_knots"] = MaxSpeedKnots,
        ["minimum_observed_distance_m"] = MinimumDistance, ["minimum_target_clearance_m"] = MinimumClearance,
        ["elapsed_s"] = Elapsed, ["in_band_s"] = InBandTime, ["following_s"] = FollowingTime,
        ["too_close_s"] = TooCloseTime, ["too_far_s"] = TooFarTime,
        ["task_elapsed_s"] = TaskTime, ["front_s"] = FrontTime, ["first_entry_s"] = FirstEntryTime,
        ["longest_following_s"] = LongestFollowingTime,
        ["after_entry_s"] = AfterEntryTime,
        ["after_entry_fraction"] = AfterEntryTime > 0 ? (double?)(AfterEntryFollowingTime / AfterEntryTime) : null,
        ["in_band_fraction"] = TaskTime > 0 ? InBandTime / TaskTime : 0,
        ["following_fraction"] = TaskTime > 0 ? FollowingTime / TaskTime : 0,
        ["obstacles_enabled"] = ObstaclesEnabled,
        ["planned_target_safety_clearance_m"] = Scenario.PlannedTargetSafetyClearance,
        ["measurement"] = "Range durations/minimum: relative straight sweep per physical step. Rear-sector classification/first entry/continuous duration: physical-step endpoints using current reference direction. Task times exclude STOP; swept safety minima include STOP." };
    private LineRenderer MakeLine(string name, Color color, int count, bool loop)
    {
        var go = new GameObject(name); go.transform.SetParent(visualRoot, false);
        var line = go.AddComponent<LineRenderer>(); line.sharedMaterial = material;
        line.useWorldSpace = true; line.loop = loop; line.positionCount = count;
        line.startColor = line.endColor = color; line.startWidth = line.endWidth = 0.8f; line.sortingOrder = 4;
        return line;
    }
    private void CreateVisuals()
    {
        visualRoot = new GameObject("FollowingTargetVisuals").transform;
        material = new Material(Shader.Find("Sprites/Default"));
        hull = MakeLine("Target hull", Color.magenta, 64, true);
        minimumRing = MakeLine("Minimum rear arc", new Color(1, .3f, .3f, .7f), 97, false);
        maximumRing = MakeLine("Maximum rear arc", new Color(.2f, 1, .6f, .7f), 97, false);
        leftEdge = MakeLine("Left sector edge", Color.cyan, 2, false);
        rightEdge = MakeLine("Right sector edge", Color.cyan, 2, false);
        arrow = MakeLine("Target direction", Color.magenta, 4, false);
        reference = MakeLine("Soft astern midpoint", Color.cyan, 5, false);
        trail = MakeLine("Observed target trail", new Color(1, .3f, 1, .5f), 0, false);
    }
    private void Draw()
    {
        if (visualRoot == null) return;
        foreach (var line in new[] { hull, minimumRing, maximumRing })
        {
            float radius = line == hull ? Scenario.target.radius_m : line == minimumRing ? MinDistance : MaxDistance;
            for (int i = 0; i < line.positionCount; i++)
            {
                var radial = line == hull ? new Vector2(Mathf.Cos(2 * Mathf.PI * i / line.positionCount), Mathf.Sin(2 * Mathf.PI * i / line.positionCount))
                    : Direction(ReferenceCourse + 90 + 180f * i / (line.positionCount - 1));
                line.SetPosition(i, (Vector3)(Position + radial * radius));
            }
        }
        var direction = Direction(Course); var side = new Vector2(direction.y, -direction.x);
        var sectorSide = Direction(ReferenceCourse + 90);
        leftEdge.SetPositions(new[] { (Vector3)(Position - sectorSide * MinDistance), (Vector3)(Position - sectorSide * MaxDistance) });
        rightEdge.SetPositions(new[] { (Vector3)(Position + sectorSide * MinDistance), (Vector3)(Position + sectorSide * MaxDistance) });
        var tip = Position + direction * (Scenario.target.radius_m + 12);
        arrow.SetPositions(new[] { (Vector3)Position, (Vector3)tip, (Vector3)(tip - direction * 5 + side * 4), (Vector3)tip });
        var p = Position - Direction(ReferenceCourse) * ((MinDistance + MaxDistance) / 2);
        reference.SetPositions(new[] { (Vector3)(p - Vector2.right * 5), (Vector3)(p + Vector2.right * 5),
            (Vector3)p, (Vector3)(p - Vector2.up * 5), (Vector3)(p + Vector2.up * 5) });
        if (history.Count == 0 || Vector3.Distance(history[history.Count - 1], Position) >= 1)
        { if (history.Count >= 6000) history.RemoveAt(0); history.Add(Position); trail.positionCount = history.Count; trail.SetPositions(history.ToArray()); }
    }
    public void ClearVisuals()
    {
        if (visualRoot != null) { visualRoot.gameObject.SetActive(false); Destroy(visualRoot.gameObject); }
        if (material != null) Destroy(material);
        visualRoot = null; material = null; history.Clear();
    }
    void OnDestroy() { ClearVisuals(); }
}
