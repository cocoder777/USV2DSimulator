using System;
using System.Collections.Generic;
using Newtonsoft.Json.Linq;
using UnityEngine;

// Task 6. Position capture ends the experiment; the four raw errors remain visible.
[DisallowMultipleComponent]
public sealed class SpecialTrackTarget : MonoBehaviour
{
    public const int PlatformId = 3001;
    public const uint TargetBatch = 30015;
    public FollowingScenarioData Scenario { get; private set; }
    public int ScenarioIndex { get; private set; }
    public float ExpectedAngle { get; private set; }
    public float expectedImpactSpeedKnots { get; private set; } = 25;
    public float ExpectedDistance { get; private set; } = 15;
    public float TargetSpeedKnots { get; private set; } = 10;
    public float InitialCourse { get; private set; }
    public Vector2 Position { get; private set; }
    public float Course { get; private set; }
    public float Speed { get; private set; }
    public double Elapsed { get; private set; }
    public bool Completed { get; private set; }
    public bool InTerminalApproach { get; private set; }
    public float Distance { get; private set; }
    public float Bearing { get; private set; }
    public float BearingError { get; private set; }
    public float DistanceError => Distance - ExpectedDistance;
    public float SpeedError => boat.Speed / DiscreteAction.MetresPerSecondPerKnot - expectedImpactSpeedKnots;
    public float PointingError { get; private set; }
    public float PositionError { get; private set; }
    public float MinimumClearance { get; private set; }
    public float CaptureRadius => boat.CollisionRadius;
    public Vector2 GoalPosition => Position + FollowingTarget.Direction(Course + ExpectedAngle) * ExpectedDistance;
    public string Status => Completed ? "Position reached; inspect terminal errors" : "Approaching terminal position";
    private BoatDynamics2D boat;
    private GameObject visuals;
    private Material material;
    private LineRenderer hull, arrow, capture, desiredArrow, trail;
    private readonly List<Vector3> history = new();

    void Awake() { boat = GetComponent<BoatDynamics2D>(); }

    public bool Configure(float angle, float speed, float distance, float targetSpeed, float course)
    {
        if (Array.Exists(new[] { angle, speed, distance, targetSpeed, course }, v => !BoatDynamics2D.IsFinite(v))
            || angle < 0 || angle >= 360 || speed < 0 || speed > boat.maxForwardSpeed / DiscreteAction.MetresPerSecondPerKnot
            || distance <= 0 || targetSpeed < 0 || targetSpeed > 200) return false;
        ExpectedAngle = angle; expectedImpactSpeedKnots = speed; ExpectedDistance = distance;
        TargetSpeedKnots = targetSpeed; InitialCourse = BoatDynamics2D.NormalizeDegrees(course);
        return GetComponent<MissionManager>().SelectSpecialTrackScenario(ScenarioIndex);
    }

    public FollowingScenarioData BuildScenario(int index)
    {
        if (ExpectedDistance <= 0
            || expectedImpactSpeedKnots > boat.maxForwardSpeed / DiscreteAction.MetresPerSecondPerKnot)
            throw new FormatException("Special-track distance must be positive and speed within current capability settings.");
        float along = ExpectedDistance * Mathf.Cos(ExpectedAngle * Mathf.Deg2Rad);
        float across = ExpectedDistance * Mathf.Sin(ExpectedAngle * Mathf.Deg2Rad);
        var scenario = MovingTargetScenarios.Build(boat, index, InitialCourse, TargetSpeedKnots, 0,
            along, along, across, across, 120, earlyManeuvers: true);
        scenario.scenario_id = "special_track_" + index;
        return scenario;
    }

    public void Load(FollowingScenarioData scenario, int index) { Scenario = scenario; ScenarioIndex = index; }

    public void ResetTarget()
    {
        ClearVisuals();
        Position = new Vector2(Scenario.target.x, Scenario.target.y); Course = Scenario.target.course_deg;
        Speed = Scenario.target.speed_knots * DiscreteAction.MetresPerSecondPerKnot;
        Elapsed = 0; Completed = false; InTerminalApproach = false; MinimumClearance = float.PositiveInfinity;
        Measure(); CreateVisuals(); Draw();
    }

    public string Step(float dt, Vector2 previousBoat, Vector2 currentBoat, bool stopping)
    {
        var previousTarget = Position; var position = Position;
        float speed = Speed, course = Course; double elapsed = Elapsed;
        Scenario.AdvanceTarget(ref position, ref speed, ref course, ref elapsed, dt);
        Position = position; Speed = speed; Course = course; Elapsed = elapsed;
        Measure();
        float clearance = DynamicObstacleField.SweptDistance(previousBoat - previousTarget, currentBoat - Position)
            - boat.CollisionRadius - Scenario.target.radius_m;
        MinimumClearance = Mathf.Min(MinimumClearance, clearance);
        // Collision always wins. A nonzero-speed capture freezes without braking.
        if (clearance <= 0) { Draw(); return "special_target_3001"; }
        if (!stopping && PositionError <= CaptureRadius) Completed = true;
        Draw(); return null;
    }

    private void Measure()
    {
        var relative = boat.Position - Position;
        Distance = relative.magnitude;
        Bearing = BoatDynamics2D.NormalizeDegrees(Mathf.Atan2(relative.x, relative.y) * Mathf.Rad2Deg - Course);
        BearingError = Mathf.DeltaAngle(ExpectedAngle, Bearing);
        float pointing = BoatDynamics2D.NormalizeDegrees(Mathf.Atan2(-relative.x, -relative.y) * Mathf.Rad2Deg);
        PointingError = Mathf.DeltaAngle(pointing, BoatDynamics2D.NormalizeDegrees(90 - boat.HeadingDeg));
        PositionError = Vector2.Distance(boat.Position, GoalPosition);
    }

    public JObject Parameters() => new JObject { ["platCode"] = 1001, ["targetBatch"] = TargetBatch,
        ["expectedImpactAngleDeg"] = ExpectedAngle, ["expectedImpactSpeed"] = expectedImpactSpeedKnots, ["expectedDistance"] = ExpectedDistance };

    public JObject Observe()
    {
        var record = ObservationBuilder.Record(PlatformId, ObservationBuilder.LonLat(Position.x, Position.y),
            Speed / DiscreteAction.MetresPerSecondPerKnot, Speed < 1e-6 ? (double?)null : Course);
        record["batch"] = TargetBatch; return record;
    }

    public JObject Diagnostics() => new JObject { ["distance_error_m"] = DistanceError,
        ["bearing_error_deg"] = BearingError, ["speed_error_knots"] = SpeedError,
        ["pointing_error_deg"] = PointingError, ["position_error_m"] = PositionError,
        ["position_capture_radius_m"] = CaptureRadius, ["position_reached"] = Completed,
        ["four_conditions_accepted"] = false };

    // The decision service owns the approach phase; capture remains a separate measurement.
    public void SetTerminalApproach(bool active) { InTerminalApproach = active; }

    public JObject StatusParams() => new JObject { ["targetBatch"] = TargetBatch,
        ["relativeDistanceM"] = Distance, ["relativeBearingDeg"] = Bearing,
        ["angleErrorDeg"] = Mathf.Abs(BearingError), ["ownSpeed"] = boat.Speed / DiscreteAction.MetresPerSecondPerKnot,
        ["speedError"] = SpeedError, ["targetSpeed"] = Speed / DiscreteAction.MetresPerSecondPerKnot,
        ["targetCourse"] = Course, ["inTerminal"] = InTerminalApproach ? 1 : 0,
        ["unity_only"] = Diagnostics() };

    public JObject Snapshot() => new JObject { ["platform_id"] = PlatformId, ["batch"] = TargetBatch,
        ["x"] = Position.x, ["y"] = Position.y, ["speed_mps"] = Speed, ["course_deg"] = Course,
        ["radius_m"] = Scenario.target.radius_m, ["goal_x"] = GoalPosition.x, ["goal_y"] = GoalPosition.y,
        ["distance_m"] = Distance, ["bearing_deg"] = Bearing, ["errors"] = Diagnostics() };

    public JObject Metrics() => new JObject { ["parameters"] = Parameters(), ["elapsed_s"] = Elapsed,
        ["result"] = Completed ? "position_reached" : "not_reached", ["terminal_errors"] = Diagnostics(),
        ["minimum_target_clearance_m"] = MinimumClearance,
        ["measurement"] = "Physical-step endpoints. Freeze on position error <= own hull radius; four-condition acceptance is not asserted. Hull collision swept per physical step and takes priority. No duration-based success." };

    private LineRenderer Line(string name, Color color, int count, bool loop)
    {
        var go = new GameObject(name); go.transform.SetParent(visuals.transform, false);
        var line = go.AddComponent<LineRenderer>(); line.sharedMaterial = material; line.useWorldSpace = true;
        line.loop = loop; line.positionCount = count; line.startColor = line.endColor = color;
        line.startWidth = line.endWidth = .7f; line.sortingOrder = 4; return line;
    }
    private void CreateVisuals()
    {
        visuals = new GameObject("SpecialTrackVisuals"); material = new Material(Shader.Find("Sprites/Default"));
        hull = Line("Target hull", Color.magenta, 48, true);
        arrow = Line("Target bow", Color.magenta, 4, false);
        capture = Line("Terminal position capture", Color.cyan, 48, true);
        desiredArrow = Line("Terminal bow pointing at target", Color.cyan, 4, false);
        trail = Line("Target observed trail", new Color(1, .2f, .8f, .5f), 0, false);
    }
    private void Draw()
    {
        if (visuals == null) return;
        var f = FollowingTarget.Direction(Course); var right = new Vector2(f.y, -f.x);
        var tip = Position + f * 20;
        arrow.SetPositions(new[] { (Vector3)Position, (Vector3)tip, (Vector3)(tip-f*5+right*4), (Vector3)tip });
        for (int i = 0; i < hull.positionCount; i++)
        {
            float a = 2*Mathf.PI*i/hull.positionCount;
            var d = new Vector2(Mathf.Cos(a), Mathf.Sin(a));
            hull.SetPosition(i, Position + d * Scenario.target.radius_m);
            capture.SetPosition(i, GoalPosition + d * CaptureRadius);
        }
        var inward = (Position - GoalPosition).normalized; var side = new Vector2(inward.y, -inward.x);
        tip = GoalPosition + inward * Mathf.Min(10, ExpectedDistance * .6f);
        desiredArrow.SetPositions(new[] { (Vector3)GoalPosition, (Vector3)tip, (Vector3)(tip-inward*3+side*2), (Vector3)tip });
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
