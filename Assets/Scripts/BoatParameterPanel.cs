
using UnityEngine;

public class BoatParameterPanel : MonoBehaviour
{
    private BoatDynamics2D boat;
    private MissionManager mission;
    private Vector2 scrollPosition;
    private float simulationSpeed = 1f;
    private float previousTimeScale;
    private string minimumDistance = "100", maximumDistance = "200", followingMaxSpeed = "0", followingError;
    private string alongMin = "-160", alongMax = "-140", acrossMin = "70", acrossMax = "90";
    private string accompanyMaxSpeed = "0", targetSpeed = "10", referenceCourse = "0", accompanyError;
    private bool accompanySettingsApplied;
    private bool syncHeading = true;
    private static readonly string[] mapButtons = { "Level 0", "Level 1", "Level 2", "Level 3", "Level 4" };
    private static readonly string[] decisionModes = { "Paused", "Continuous" };
    private static readonly string[] mapDescriptions = {
        "Original route / 3 checkpoints",
        "Scattered route / 5 checkpoints",
        "Alternating bends / 7 checkpoints",
        "Return bends / 9 checkpoints",
        "Dense crossings / 9 checkpoints"
    };

    private GUIStyle titleStyle;
    private GUIStyle labelStyle;
    private GUIStyle hintStyle;

    private Rect panelRect;

    void Awake()
    {
        boat = GetComponent<BoatDynamics2D>();
        mission = GetComponent<MissionManager>();
    }

    void OnEnable()
    {
        previousTimeScale = Time.timeScale;
        Time.timeScale = simulationSpeed;
    }

    void OnDisable()
    {
        Time.timeScale = previousTimeScale;
    }

    void OnGUI()
    {
        float guiScale = Mathf.Clamp(Screen.width / 1280f, 0.4f, 1.5f);
        var previousMatrix = GUI.matrix;
        GUI.matrix = Matrix4x4.Scale(new Vector3(guiScale, guiScale, 1));
        if (boat == null) { GUI.matrix = previousMatrix; return; }

        // 面板固定在屏幕右上角
        panelRect = new Rect(
            Mathf.Max(10f, (Screen.width / guiScale) - 330f),
            10f,
            Mathf.Min(320f, (Screen.width / guiScale) - 20f),
            Mathf.Max(160f, Mathf.Min(570f, (Screen.height / guiScale) - 70f))
        );

        GUI.Box(panelRect, GUIContent.none);

        GUILayout.BeginArea(new Rect(
            panelRect.x + 12f,
            panelRect.y + 10f,
            panelRect.width - 24f,
            panelRect.height - 20f
        ));

        InitializeStyles();

        scrollPosition = GUILayout.BeginScrollView(scrollPosition);

        GUILayout.Label("SIMULATION SPEED", titleStyle);
        float requestedSpeed = DrawSlider("Simulation speed", simulationSpeed, 1f, 10f, "x");
        requestedSpeed = Mathf.Round(requestedSpeed * 10f) / 10f;
        if (GUILayout.Button("Reset speed to 1x", GUILayout.Height(24))) requestedSpeed = 1f;
        if (!Mathf.Approximately(requestedSpeed, simulationSpeed))
        {
            simulationSpeed = requestedSpeed;
            // Run more fixed steps per real second without changing integration precision.
            Time.timeScale = simulationSpeed;
        }
        GUILayout.Label("Speeds up the boat and obstacles. Waiting for Jev still pauses simulation.", hintStyle);
        GUILayout.Space(8);

        if (mission != null)
        {
            GUILayout.Label("TASK MODE", titleStyle);
            int taskMode = GUILayout.SelectionGrid((int)mission.Mode, new[] { "Route", "Following", "Accompany" }, 3);
            if (taskMode != (int)mission.Mode) mission.SelectTaskMode((MissionManager.TaskMode)taskMode);
            if (mission.IsAccompany)
            {
                var a = mission.Accompany;
                GUILayout.Label("FRIENDLY ACCOMPANY", titleStyle);
                int scenario = GUILayout.SelectionGrid(a.ScenarioIndex, AccompanyTarget.ScenarioNames, 2);
                if (scenario != a.ScenarioIndex) mission.SelectAccompanyScenario(scenario);
                GUILayout.Label("Own 1001 / friendly target 3001. No obstacles.", hintStyle);
                bool changedBeforeSettings = GUI.changed;
                GUI.changed = false;
                DrawRange("Along min / max (m)", ref alongMin, ref alongMax);
                DrawRange("Across min / max (m)", ref acrossMin, ref acrossMax);
                GUILayout.Label("Along: +forward / -behind. Across: +right / -left.", hintStyle);
                syncHeading = GUILayout.Toggle(syncHeading, "Sync heading (straight / stopped)");
                GUILayout.Label("Task maxSpeed (knots; 0 = physical limit)", hintStyle);
                accompanyMaxSpeed = GUILayout.TextField(accompanyMaxSpeed);
                GUILayout.Label("Target speed (knots) / initial direction (deg)", hintStyle);
                GUILayout.BeginHorizontal(); targetSpeed = GUILayout.TextField(targetSpeed); referenceCourse = GUILayout.TextField(referenceCourse); GUILayout.EndHorizontal();
                if (GUI.changed) { accompanySettingsApplied = false; accompanyError = null; }
                GUI.changed |= changedBeforeSettings;
                GUILayout.Label("Edit bounds, then Apply to move / resize the cyan band and reset.", hintStyle);
                if (GUILayout.Button("Apply accompany settings and reset"))
                {
                    accompanyError = TryNumber(alongMin, out var amin) && TryNumber(alongMax, out var amax)
                        && TryNumber(acrossMin, out var cmin) && TryNumber(acrossMax, out var cmax)
                        && TryNumber(accompanyMaxSpeed, out var maximum) && TryNumber(targetSpeed, out var speed)
                        && TryNumber(referenceCourse, out var course) && a.Configure(amin, amax, cmin, cmax, syncHeading, maximum, speed, course)
                        ? null : "Use finite ordered bounds outside target hull, nonnegative speeds.";
                    accompanySettingsApplied = accompanyError == null;
                }
                if (accompanyError != null) GUILayout.Label(accompanyError, hintStyle);
                else if (accompanySettingsApplied) GUILayout.Label("Settings applied. Task reset.", hintStyle);
                GUILayout.Label($"Active along [{a.AlongMin:F2}, {a.AlongMax:F2}] m", labelStyle);
                GUILayout.Label($"Active across [{a.AcrossMin:F2}, {a.AcrossMax:F2}] m", labelStyle);
                GUILayout.Label($"syncHeading={(a.SyncHeading ? 1 : 0)}, maxSpeed={a.MaxSpeedKnots:F1} knots", labelStyle);
                GUILayout.Label($"Along {a.Along:F1}, across {a.Across:F1} m", labelStyle);
                GUILayout.Label($"{a.Status} | position rate {a.PositionRate:F2} m/s", labelStyle);
                GUILayout.Label($"Heading error {a.HeadingError:F1} deg", labelStyle);
                GUILayout.Label("Turns: position first; align when straight.", hintStyle);
                GUILayout.Label($"Elapsed {a.Elapsed:F1}/{a.Scenario.duration_s:F0} s", labelStyle);
                GUILayout.Label($"In position {(a.TaskTime > 0 ? a.InPositionTime/a.TaskTime : 0):P0}", labelStyle);
                GUILayout.Label("Green: friendly target. Cyan: allowed position band. Starts outside the band.", hintStyle);
            }
            else if (mission.IsFollowing)
            {
                var follow = mission.Following;
                GUILayout.Label("ENEMY FOLLOWING", titleStyle);
                int scenario = GUILayout.SelectionGrid(follow.ScenarioIndex, FollowingTarget.ScenarioNames, 2);
                if (scenario != follow.ScenarioIndex) mission.SelectFollowingScenario(scenario);
                if (follow.Scenario.obstacles.Length > 0)
                {
                    bool enabled = GUILayout.Toggle(follow.ObstaclesEnabled, "Scenario obstacles (switching resets)");
                    if (enabled != follow.ObstaclesEnabled) follow.SetObstaclesEnabled(enabled);
                    GUILayout.Label($"Target route safety margin: {follow.Scenario.PlannedTargetSafetyClearance:F1} m", hintStyle);
                }
                GUILayout.Label("Center-to-center distance (metres)", hintStyle);
                GUILayout.BeginHorizontal();
                GUILayout.Label("Min", GUILayout.Width(30)); minimumDistance = GUILayout.TextField(minimumDistance, GUILayout.Width(80));
                GUILayout.Label("Max", GUILayout.Width(30)); maximumDistance = GUILayout.TextField(maximumDistance, GUILayout.Width(80));
                GUILayout.EndHorizontal();
                GUILayout.Label("Task maxSpeed (knots; 0 = physical limit)", hintStyle);
                followingMaxSpeed = GUILayout.TextField(followingMaxSpeed);
                if (GUILayout.Button("Apply distances and reset"))
                {
                    var culture = System.Globalization.CultureInfo.InvariantCulture;
                    followingError = float.TryParse(minimumDistance, System.Globalization.NumberStyles.Float, culture, out var min)
                        && float.TryParse(maximumDistance, System.Globalization.NumberStyles.Float, culture, out var max)
                        && TryNumber(followingMaxSpeed, out var cap)
                        && follow.ConfigureDistances(min, max, cap) ? null : "Use 0 <= min <= max, max > combined hull radii, maxSpeed >= 0.";
                }
                if (followingError != null) GUILayout.Label(followingError, hintStyle);
                GUILayout.Label($"Active range: {follow.MinDistance:F1} - {follow.MaxDistance:F1} m", labelStyle);
                GUILayout.Label($"Target: {follow.Speed / DiscreteAction.MetresPerSecondPerKnot:F1} knots | {follow.TrackingStatus}", labelStyle);
                GUILayout.Label($"Range: {follow.Distance:F1} m | changing {follow.RangeRate:F2} m/s", labelStyle);
                GUILayout.Label($"Bearing: {follow.RelativeBearing:F1} deg | allowed 90 - 270", labelStyle);
                GUILayout.Label($"Elapsed: {follow.Elapsed:F1} / {follow.Scenario.duration_s:F0} s | rear sector: {(follow.TaskTime > 0 ? follow.FollowingTime / follow.TaskTime : 0):P0}", labelStyle);
                GUILayout.Label($"Range only: {(follow.TaskTime > 0 ? follow.InBandTime / follow.TaskTime : 0):P0} | longest hold {follow.LongestFollowingTime:F1} s", labelStyle);
                GUILayout.Label("Magenta: target/trail. Rear arcs: allowed region. Cyan cross: soft astern midpoint.", hintStyle);
            }
            else
            {
            GUILayout.Label("MAP LEVEL", titleStyle);
            int selected = string.IsNullOrWhiteSpace(mission.routeFile) ? mission.MapLevel : -1;
            int next = GUILayout.SelectionGrid(selected, mapButtons, 3, GUILayout.Height(52));
            if (next >= 0 && next != selected) mission.SelectMapLevel(next);
            GUILayout.Label(string.IsNullOrWhiteSpace(mission.routeFile)
                ? mapDescriptions[mission.MapLevel] : "Custom task", labelStyle);
            GUILayout.Label("Switching maps resets the boat and task.", hintStyle);
            }
            if (!string.IsNullOrEmpty(mission.LoadError)) GUILayout.Label(mission.LoadError, hintStyle);
            GUILayout.Space(8);
        }

        var runner = GetComponent<SimulationRunner>();
        if (runner != null)
        {
            GUILayout.Label("JEV CONTROL", titleStyle);
            bool enabled = GUI.enabled;
            GUILayout.Label($"Decision mode: {runner.DecisionMode}", labelStyle);
            GUI.enabled = enabled && runner.CanChangeDecisionMode && !runner.JevStarted;
            int nextMode = GUILayout.SelectionGrid((int)runner.DecisionMode, decisionModes, 2);
            if (nextMode != (int)runner.DecisionMode)
                runner.TrySetDecisionMode((SimulationRunner.DecisionTimingMode)nextMode);
            GUI.enabled = enabled;
            GUILayout.Label("Simulation pauses while waiting for Jev. Mode switching is not available yet.", hintStyle);
            GUI.enabled = enabled && !runner.JevStarted;
            float requestedDuration = DrawSlider("Action duration", runner.actionDuration,
                Time.fixedDeltaTime, 2f, "s");
            if (!runner.JevStarted && !Mathf.Approximately(requestedDuration, runner.actionDuration))
                runner.actionDuration = Mathf.Round(requestedDuration / Time.fixedDeltaTime) * Time.fixedDeltaTime;
            GUI.enabled = enabled;
            GUILayout.Label("Set duration before starting. Reset to change it.", hintStyle);
            if (boat.Mode == BoatDynamics2D.ControlMode.Jev && !runner.JevStarted
                && GUILayout.Button("Start Jev", GUILayout.Height(28))) runner.StartJev();
            GUI.enabled = enabled && boat.Mode == BoatDynamics2D.ControlMode.Jev
                && runner.JevStarted && !runner.StopRequested && mission != null && mission.HasTask
                && !mission.Completed;
            if (GUILayout.Button("Stop via Jev", GUILayout.Height(28))) runner.RequestImmediateStop();
            GUI.enabled = enabled;
            if (runner.StopRequested) GUILayout.Label("Jev stop requested: braking in place.", hintStyle);
            var camera = Camera.main?.GetComponent<BoatCameraFollow>();
            if (camera != null) camera.overview = GUILayout.Toggle(camera.overview, "Map overview (mouse wheel to zoom)");
            GUILayout.Space(8);
        }
        var trajectory = GetComponent<BoatTrajectoryRecorder>();
        if (trajectory != null)
        {
            if (trajectory.IsPlotting) GUILayout.Label("Generating trajectory plot...", hintStyle);
            if (!string.IsNullOrEmpty(trajectory.LastImagePath)
                && GUILayout.Button("Open trajectory plot", GUILayout.Height(28))) trajectory.OpenPlot();
            if (!string.IsNullOrEmpty(trajectory.ExportError))
            {
                GUILayout.Label(trajectory.ExportError, hintStyle);
                if (trajectory.LastDataPath != null && !trajectory.IsPlotting
                    && GUILayout.Button("Retry trajectory plot")) trajectory.RetryPlot();
            }
        }
        GUILayout.Label("USV PARAMETERS", titleStyle);
        GUILayout.Label("Changes apply immediately.", hintStyle);
        GUILayout.Space(8);

        boat.maxForwardSpeed = DrawSlider("Max forward speed",
            boat.maxForwardSpeed / DiscreteAction.MetresPerSecondPerKnot, 1f, 100f, "knots")
            * DiscreteAction.MetresPerSecondPerKnot;
        boat.maxReverseSpeed = DrawSlider("Max reverse speed", boat.maxReverseSpeed, 0f, 3f, "m/s");
        boat.maxAcceleration = DrawSlider("Max acceleration", boat.maxAcceleration, 0.1f, 5f, "m/s^2");
        boat.maxDeceleration = DrawSlider("Max deceleration", boat.maxDeceleration, 0.1f, 5f, "m/s^2");
        boat.maxAngularAccelerationDeg = DrawSlider("Max angular acceleration",
            boat.maxAngularAccelerationDeg, 0.1f, 20f, "deg/s^2");
        boat.maxYawRateDeg = DrawSlider("Max turning rate", boat.maxYawRateDeg, 0.1f, 30f, "deg/s");
        boat.lengthMetres = DrawSlider("Hull length", boat.lengthMetres, 1f, 30f, "m");
        boat.beamMetres = DrawSlider("Hull beam", boat.beamMetres, 0.5f, 10f, "m");
        GUILayout.Label($"Circular hull radius: {boat.CollisionRadius:F2} m (collision)", hintStyle);
        if (mission == null || !mission.HasMovingTarget)
        {
            boat.waypointRadiusMultiplier = DrawSlider("Waypoint radius multiplier", boat.waypointRadiusMultiplier, 1f, 20f, "x");
            GUILayout.Label($"Waypoint acceptance radius: {boat.WaypointRadius:F2} m", hintStyle);
        }
        GUILayout.Label("Speed and turn rates are acceleration-limited. Settings apply in Manual and Jev modes.", hintStyle);
        var obstacles = GetComponent<DynamicObstacleField>();
        if (obstacles != null && (mission == null || !mission.IsAccompany))
        {
            obstacles.smallAvoidanceRadius = DrawSlider("Small target avoidance", obstacles.smallAvoidanceRadius, 10f, 100f, "m");
            obstacles.largeAvoidanceRadius = DrawSlider("Large target avoidance", obstacles.largeAvoidanceRadius, 80f, 100f, "m");
        }

        GUILayout.Space(6);

        if (GUILayout.Button(
            "Restore Default Parameters",
            GUILayout.Height(28)
        ))
        {
            RestoreDefaults();
        }

        GUILayout.Space(4);
        if (boat.Mode == BoatDynamics2D.ControlMode.Manual)
            GUILayout.Label("Response = 0 means no input filtering.", hintStyle);
        GUILayout.Label(
            "W/S: accelerate   A/D: turn   R: reset boat",
            hintStyle
        );

        GUILayout.EndScrollView();
        GUILayout.EndArea();
        GUI.matrix = previousMatrix;
    }

    void InitializeStyles()
    {
        if (titleStyle != null)
            return;

        titleStyle = new GUIStyle(GUI.skin.label);
        titleStyle.fontSize = 17;
        titleStyle.fontStyle = FontStyle.Bold;
        titleStyle.normal.textColor = Color.cyan;

        labelStyle = new GUIStyle(GUI.skin.label);
        labelStyle.fontSize = 13;
        labelStyle.normal.textColor = Color.white;

        hintStyle = new GUIStyle(GUI.skin.label);
        hintStyle.fontSize = 11;
        hintStyle.normal.textColor = Color.gray;
        hintStyle.wordWrap = true;
    }

    void DrawRange(string label, ref string minimum, ref string maximum)
    {
        GUILayout.Label(label, labelStyle);
        GUILayout.BeginHorizontal();
        GUILayout.Label("Min", GUILayout.Width(28));
        minimum = GUILayout.TextField(minimum, GUILayout.MinWidth(60), GUILayout.ExpandWidth(true));
        GUILayout.Label("Max", GUILayout.Width(28));
        maximum = GUILayout.TextField(maximum, GUILayout.MinWidth(60), GUILayout.ExpandWidth(true));
        GUILayout.EndHorizontal();
    }
    static bool TryNumber(string text, out float value) => float.TryParse(text, System.Globalization.NumberStyles.Float,
        System.Globalization.CultureInfo.InvariantCulture, out value);

    float DrawSlider(
        string label,
        float value,
        float min,
        float max,
        string unit
    )
    {
        GUILayout.Label(
            $"{label}: {value:F2} {unit}",
            labelStyle,
            GUILayout.Height(19)
        );

        return GUILayout.HorizontalSlider(
            value, min, max, GUILayout.Height(18)
        );
    }

    void RestoreDefaults()
    {
        BoatParameters.Restore(boat);
        var field = GetComponent<DynamicObstacleField>();
        if (field != null) field.RestoreDefaults();
    }
}
