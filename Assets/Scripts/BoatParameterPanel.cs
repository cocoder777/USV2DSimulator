
using UnityEngine;

public class BoatParameterPanel : MonoBehaviour
{
    private BoatDynamics2D boat;
    private MissionManager mission;
    private Vector2 scrollPosition;
    private float simulationSpeed = 1f;
    private float previousTimeScale;
    private string minimumDistance = "100", maximumDistance = "200", followingError;
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
            int taskMode = GUILayout.SelectionGrid((int)mission.Mode, new[] { "Route following", "Distance following" }, 2);
            if (taskMode != (int)mission.Mode) mission.SelectTaskMode((MissionManager.TaskMode)taskMode);
            if (mission.IsFollowing)
            {
                var follow = mission.Following;
                GUILayout.Label("FOLLOWING MAP", titleStyle);
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
                if (GUILayout.Button("Apply distances and reset"))
                {
                    var culture = System.Globalization.CultureInfo.InvariantCulture;
                    followingError = float.TryParse(minimumDistance, System.Globalization.NumberStyles.Float, culture, out var min)
                        && float.TryParse(maximumDistance, System.Globalization.NumberStyles.Float, culture, out var max)
                        && follow.ConfigureDistances(min, max) ? null : "Use finite min > combined hull radii, max > min.";
                }
                if (followingError != null) GUILayout.Label(followingError, hintStyle);
                GUILayout.Label($"Active range: {follow.MinDistance:F1} - {follow.MaxDistance:F1} m", labelStyle);
                GUILayout.Label($"Target: {follow.Speed / DiscreteAction.MetresPerSecondPerKnot:F1} knots | {follow.TrackingStatus}", labelStyle);
                GUILayout.Label($"Range: {follow.Distance:F1} m | changing {follow.RangeRate:F2} m/s", labelStyle);
                GUILayout.Label($"Elapsed: {follow.Elapsed:F1} / {follow.Scenario.duration_s:F0} s | in band: {(follow.Elapsed > 0 ? follow.InBandTime / follow.Elapsed : 0):P0}", labelStyle);
                GUILayout.Label("Magenta: target/trail. Red/green: distance limits. Cyan: reference on the boat-target line.", hintStyle);
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
        if (mission == null || !mission.IsFollowing)
        {
            boat.waypointRadiusMultiplier = DrawSlider("Waypoint radius multiplier", boat.waypointRadiusMultiplier, 1f, 20f, "x");
            GUILayout.Label($"Waypoint acceptance radius: {boat.WaypointRadius:F2} m", hintStyle);
        }
        GUILayout.Label("Speed and turn rates are acceleration-limited. Settings apply in Manual and Jev modes.", hintStyle);
        var obstacles = GetComponent<DynamicObstacleField>();
        if (obstacles != null)
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
