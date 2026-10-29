using System;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;

// Task 3: an independent friendly actor, reusing the existing target motion integrator.
[DisallowMultipleComponent]
public sealed class AccompanyTarget : MonoBehaviour
{
    public const int PlatformId = 3001;
    public static readonly string[] ScenarioNames = { "Straight", "Gentle turn", "Speed / stop", "Irregular" };
    public FollowingScenarioData Scenario { get; private set; }
    public int ScenarioIndex { get; private set; }
    public float AlongMin { get; private set; } = -160;
    public float AlongMax { get; private set; } = -140;
    public float AcrossMin { get; private set; } = 70;
    public float AcrossMax { get; private set; } = 90;
    public bool SyncHeading { get; private set; } = true;
    public float MaxSpeedKnots { get; private set; } = 0;
    public float TargetSpeedKnots { get; private set; } = 10;
    public float InitialReferenceCourse { get; private set; } = 0;
    public float DurationSeconds { get; set; } = 120;
    public Vector2 Position { get; private set; }
    public float Course { get; private set; }
    public float Speed { get; private set; }
    public double Elapsed { get; private set; }
    public bool Completed => Scenario != null && Elapsed + Time.fixedDeltaTime / 2 >= Scenario.duration_s;
    public float Along { get; private set; }
    public float Across { get; private set; }
    public float AlongOut { get; private set; }
    public float AcrossOut { get; private set; }
    public bool InPosition => AlongOut == 0 && AcrossOut == 0;
    public float RelativeSpeed { get; private set; }
    public float PositionRate { get; private set; }
    public float HeadingError { get; private set; }
    public float TargetYawRate { get; private set; }
    public bool HeadingSyncActive => SyncHeading && Mathf.Abs(TargetYawRate) <= .01f;
    public float MinimumClearance { get; private set; }
    public double InPositionTime { get; private set; }
    public double StableTime { get; private set; }
    public double TaskTime { get; private set; }
    public double? FirstEntryTime { get; private set; }
    public string Status => InPosition ? "In position" : "Acquiring position";
    private BoatDynamics2D boat;
    private MissionManager mission;
    private GameObject visuals;
    private Material material;
    private LineRenderer band, hull, arrow, trail;
    private readonly List<Vector3> history = new();

    void Awake() { boat = GetComponent<BoatDynamics2D>(); mission = GetComponent<MissionManager>(); }

    public bool Configure(float alongMin, float alongMax, float acrossMin, float acrossMax,
        bool syncHeading, float maxSpeedKnots, float targetSpeedKnots = 10, float referenceCourse = 0)
    {
        foreach (float v in new[] { alongMin, alongMax, acrossMin, acrossMax, maxSpeedKnots, targetSpeedKnots, referenceCourse, DurationSeconds })
            if (!BoatDynamics2D.IsFinite(v)) return false;
        float nearestAlong = Mathf.Clamp(0, alongMin, alongMax), nearestAcross = Mathf.Clamp(0, acrossMin, acrossMax);
        if (alongMin > alongMax || acrossMin > acrossMax || maxSpeedKnots < 0
            || targetSpeedKnots < 0 || targetSpeedKnots > 200 || DurationSeconds <= 0
            || new Vector2(nearestAlong, nearestAcross).magnitude <= boat.CollisionRadius + 6) return false;
        AlongMin = alongMin; AlongMax = alongMax; AcrossMin = acrossMin; AcrossMax = acrossMax;
        SyncHeading = syncHeading; MaxSpeedKnots = maxSpeedKnots; TargetSpeedKnots = targetSpeedKnots;
        InitialReferenceCourse = BoatDynamics2D.NormalizeDegrees(referenceCourse);
        return mission.SelectAccompanyScenario(ScenarioIndex);
    }

    public FollowingScenarioData BuildScenario(int index)
    {
        if (index < 0 || index >= ScenarioNames.Length) throw new FormatException("Invalid accompany scenario.");
        if (!BoatDynamics2D.IsFinite(DurationSeconds) || DurationSeconds <= 0) throw new FormatException("Invalid experiment duration.");
        var f = FollowingTarget.Direction(InitialReferenceCourse); var right = new Vector2(f.y, -f.x);
        // Outside every configured band; no teleport-to-position start.
        float along = AlongMin - 80;
        float across = (AcrossMin + AcrossMax) / 2;
        across = across >= 0 ? AcrossMax + 80 : AcrossMin - 80;
        var initial = f * along + right * across;
        // A sustained turn must exercise more than a complete revolution after acquisition.
        float duration = index == 1 ? Mathf.Max(DurationSeconds, 360)
            : index == 3 ? Mathf.Max(DurationSeconds, 240) : DurationSeconds;
        float targetSpeed = TargetSpeedKnots;
        float targetAcceleration = 1.46f, targetDeceleration = 1.75f;
        var segments = new List<FollowingScenarioData.Segment> {
            new() { start_s = 0, speed_knots = targetSpeed, yaw_rate_deg_s = 0 }
        };
        if (index == 1) segments.Add(new() { start_s = Mathf.Min(48, duration * .4f), speed_knots = TargetSpeedKnots, yaw_rate_deg_s = 2 });
        if (index == 2)
        {
            segments.Add(new() { start_s = DurationSeconds * .35f, speed_knots = TargetSpeedKnots * .6f });
            segments.Add(new() { start_s = DurationSeconds * .6f, speed_knots = 0 });
            segments.Add(new() { start_s = DurationSeconds * .8f, speed_knots = TargetSpeedKnots });
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
            segments = BuildIrregularSegments(duration, targetSpeed, maximumYaw, yawAcceleration);
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
        float duration, float maximumSpeed, float maximumYaw, float yawAcceleration)
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
        Add(Mathf.Min(40, duration), maximumSpeed, 0);
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

    public void Load(FollowingScenarioData scenario, int index) { Scenario = scenario; ScenarioIndex = index; }

    public void ResetTarget()
    {
        ClearVisuals();
        if (!mission.IsAccompany || Scenario == null) return;
        Position = new Vector2(Scenario.target.x, Scenario.target.y); Course = Scenario.target.course_deg;
        TargetYawRate = 0;
        Speed = Scenario.target.speed_knots * DiscreteAction.MetresPerSecondPerKnot;
        Elapsed = InPositionTime = StableTime = TaskTime = 0; FirstEntryTime = null; MinimumClearance = float.PositiveInfinity;
        Measure(boat.Position, boat.Position, Position, 0);
        CreateVisuals(); Draw();
    }

    public string Step(float dt, Vector2 previousBoat, Vector2 currentBoat, bool stopping)
    {
        var previousTarget = Position; var position = Position; float speed = Speed, course = Course; double elapsed = Elapsed;
        Scenario.AdvanceTarget(ref position, ref speed, ref course, ref elapsed, dt);
        TargetYawRate = dt > 0 ? Mathf.DeltaAngle(Course, course) / dt : 0;
        Position = position; Speed = speed; Course = course; Elapsed = elapsed;
        Measure(previousBoat, currentBoat, previousTarget, dt, !stopping); Draw();
        return MinimumClearance <= 0 ? "friendly_target_3001" : null;
    }

    private void Measure(Vector2 previousBoat, Vector2 currentBoat, Vector2 previousTarget, float dt, bool countTask = true)
    {
        float previousAlong = Along, previousAcross = Across;
        var f = FollowingTarget.Direction(Course); var right = new Vector2(f.y, -f.x);
        var relative = currentBoat - Position;
        Along = Vector2.Dot(relative, f); Across = Vector2.Dot(relative, right);
        AlongOut = Along - Mathf.Clamp(Along, AlongMin, AlongMax);
        AcrossOut = Across - Mathf.Clamp(Across, AcrossMin, AcrossMax);
        PositionRate = dt > 0 ? new Vector2(Along-previousAlong, Across-previousAcross).magnitude / dt : 0;
        var ownVelocity = new Vector2(Mathf.Cos(boat.HeadingDeg * Mathf.Deg2Rad), Mathf.Sin(boat.HeadingDeg * Mathf.Deg2Rad)) * boat.Speed;
        RelativeSpeed = (ownVelocity - f * Speed).magnitude;
        HeadingError = Mathf.Abs(Mathf.DeltaAngle(90 - boat.HeadingDeg, Course));
        MinimumClearance = Mathf.Min(MinimumClearance,
            DynamicObstacleField.SweptDistance(previousBoat - previousTarget, relative) - boat.CollisionRadius - Scenario.target.radius_m);
        if (countTask) TaskTime += dt;
        if (countTask && dt > 0 && InPosition)
        {
            InPositionTime += dt;
            if (FirstEntryTime == null) FirstEntryTime = Elapsed;
            if (PositionRate <= 1 && (!HeadingSyncActive || HeadingError <= 25)) StableTime += dt;
        }
    }

    public JObject Parameters() => new JObject {
        ["platCode"] = 1001, ["targetPlatCode"] = PlatformId, ["offsetAlongMinM"] = AlongMin,
        ["offsetAlongMaxM"] = AlongMax, ["offsetAcrossMinM"] = AcrossMin, ["offsetAcrossMaxM"] = AcrossMax,
        ["syncHeading"] = SyncHeading ? 1 : 0, ["maxSpeed"] = MaxSpeedKnots };
    public JObject Observe() => ObservationBuilder.Record(PlatformId, ObservationBuilder.LonLat(Position.x, Position.y),
        Speed / DiscreteAction.MetresPerSecondPerKnot, Speed < 1e-6 ? (double?)null : Course, 2);
    public JObject StatusParams() => new JObject { ["targetPlatCode"] = PlatformId, ["currentAlongM"] = Along,
        ["currentAcrossM"] = Across, ["alongOutM"] = AlongOut, ["acrossOutM"] = AcrossOut,
        ["inPosition"] = InPosition ? 1 : 0, ["targetSpeed"] = Speed / DiscreteAction.MetresPerSecondPerKnot, ["targetCourse"] = Course };
    public JObject Snapshot() => new JObject { ["platform_id"] = PlatformId, ["x"] = Position.x, ["y"] = Position.y,
        ["speed_mps"] = Speed, ["course_deg"] = Course, ["radius_m"] = Scenario.target.radius_m,
        ["along_m"] = Along, ["across_m"] = Across, ["along_out_m"] = AlongOut, ["across_out_m"] = AcrossOut,
        ["in_position"] = InPosition, ["relative_speed_mps"] = RelativeSpeed, ["position_rate_mps"] = PositionRate, ["heading_error_deg"] = HeadingError, ["target_yaw_deg_s"] = TargetYawRate, ["heading_sync_active"] = HeadingSyncActive };
    public JObject Metrics() => new JObject { ["parameters"] = Parameters(), ["elapsed_s"] = Elapsed,
        ["first_entry_s"] = FirstEntryTime, ["in_position_s"] = InPositionTime, ["stable_s"] = StableTime,
        ["task_elapsed_s"] = TaskTime, ["in_position_fraction"] = TaskTime > 0 ? InPositionTime / TaskTime : 0,
        ["minimum_target_clearance_m"] = MinimumClearance,
        ["measurement"] = "Position/velocity/heading at physical-step endpoints; hull clearance swept per step. Stable: in band, target-frame position rate <=1m/s, heading error <=25deg when sync enabled and target straight/stopped (observed yaw <=0.01deg/s); turns prioritize position. Raw world-relative speed also recorded; these differ while target frame rotates. STOP excluded from accumulated task times." };

    private LineRenderer Line(string name, Color color, int count, bool loop)
    {
        var go = new GameObject(name); go.transform.SetParent(visuals.transform, false);
        var line = go.AddComponent<LineRenderer>(); line.sharedMaterial = material; line.useWorldSpace = true;
        line.loop = loop; line.positionCount = count; line.startColor = line.endColor = color;
        line.startWidth = line.endWidth = 1; line.sortingOrder = 4; return line;
    }
    private void CreateVisuals()
    {
        visuals = new GameObject("AccompanyVisuals"); material = new Material(Shader.Find("Sprites/Default"));
        band = Line("Allowed position band", Color.cyan, 4, true);
        hull = Line("Friendly hull", Color.green, 48, true);
        arrow = Line("Friendly direction", Color.green, 4, false);
        trail = Line("Friendly observed trail", new Color(.2f, 1, .4f, .6f), 0, false);
    }
    private void Draw()
    {
        if (visuals == null) return;
        var f = FollowingTarget.Direction(Course); var right = new Vector2(f.y, -f.x);
        band.SetPositions(new[] { (Vector3)(Position+f*AlongMin+right*AcrossMin), (Vector3)(Position+f*AlongMax+right*AcrossMin),
            (Vector3)(Position+f*AlongMax+right*AcrossMax), (Vector3)(Position+f*AlongMin+right*AcrossMax) });
        for (int i = 0; i < hull.positionCount; i++)
        { float a = 2*Mathf.PI*i/hull.positionCount; hull.SetPosition(i, Position + new Vector2(Mathf.Cos(a), Mathf.Sin(a))*Scenario.target.radius_m); }
        var tip = Position + f * 20;
        arrow.SetPositions(new[] { (Vector3)Position, (Vector3)tip, (Vector3)(tip-f*5+right*4), (Vector3)tip });
        if (history.Count == 0 || Vector3.Distance(history[history.Count-1], Position) >= 1)
        { if (history.Count >= 6000) history.RemoveAt(0); history.Add(Position); trail.positionCount = history.Count; trail.SetPositions(history.ToArray()); }
    }
    public void ClearVisuals()
    {
        if (visuals != null) { visuals.SetActive(false); Destroy(visuals); }
        if (material != null) Destroy(material);
        visuals = null; material = null; history.Clear();
    }
    void OnDestroy() { ClearVisuals(); }
}
