
using UnityEngine;

public class BoatParameterPanel : MonoBehaviour
{
    private BoatDynamics2D boat;
    private MissionManager mission;
    private Vector2 scrollPosition;
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

    void OnGUI()
    {
        if (boat == null)
            return;

        // 面板固定在屏幕右上角
        panelRect = new Rect(
            Mathf.Max(10f, Screen.width - 330f),
            10f,
            Mathf.Min(320f, Screen.width - 20f),
            Mathf.Max(160f, Mathf.Min(570f, Screen.height - 70f))
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

        if (mission != null)
        {
            GUILayout.Label("MAP LEVEL", titleStyle);
            int selected = string.IsNullOrWhiteSpace(mission.routeFile) ? mission.MapLevel : -1;
            int next = GUILayout.SelectionGrid(selected, mapButtons, 3, GUILayout.Height(52));
            if (next >= 0 && next != selected) mission.SelectMapLevel(next);
            GUILayout.Label(string.IsNullOrWhiteSpace(mission.routeFile)
                ? mapDescriptions[mission.MapLevel] : "Custom task", labelStyle);
            GUILayout.Label("Switching maps resets the boat and task.", hintStyle);
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
            float duration = Mathf.Round(DrawSlider("Action duration", runner.actionDuration,
                Time.fixedDeltaTime, 2f, "s") / Time.fixedDeltaTime) * Time.fixedDeltaTime;
            if (!runner.JevStarted) runner.actionDuration = duration;
            GUI.enabled = enabled;
            GUILayout.Label("Set duration before starting. Reset to change it.", hintStyle);
            if (boat.Mode == BoatDynamics2D.ControlMode.Jev && !runner.JevStarted
                && GUILayout.Button("Start Jev", GUILayout.Height(28))) runner.StartJev();
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

        boat.maxForwardSpeed = DrawSlider(
            "Max forward speed",
            boat.maxForwardSpeed, 0.5f, 10f, "m/s"
        );

        boat.maxReverseSpeed = DrawSlider(
            "Max reverse speed",
            boat.maxReverseSpeed, 0f, 3f, "m/s"
        );

        if (boat.Mode == BoatDynamics2D.ControlMode.Jev)
        {
            boat.speedResponseTime = DrawSlider("Speed transition time",
                boat.speedResponseTime, 0f, 5f, "s");
            boat.headingResponseTime = DrawSlider("Heading transition time", boat.headingResponseTime, 0f, 5f, "s");
            if (GUILayout.Button("Instant heading (0 s)")) boat.headingResponseTime = 0;
            if (GUILayout.Button("Instant speed (0 s)")) boat.speedResponseTime = 0;
            GUILayout.Label("Speed ramps linearly. Heading turns smoothly and arrives with zero yaw. Each time is independent; 0 is instant.", hintStyle);
        }
        else
        {
            boat.maxAcceleration = DrawSlider(
                "Max acceleration",
                boat.maxAcceleration, 0.1f, 5f, "m/s^2"
            );

            boat.maxAngularAccelerationDeg = DrawSlider(
                "Max angular acceleration",
                boat.maxAngularAccelerationDeg, 5f, 180f, "deg/s^2"
            );

            boat.maxYawRateDeg = DrawSlider(
                "Max turning rate",
                boat.maxYawRateDeg, 5f, 120f, "deg/s"
            );

            boat.accelerationTimeConstant = DrawSlider(
                "Linear acceleration response",
                boat.accelerationTimeConstant, 0f, 5f, "s"
            );

            boat.angularAccelerationTimeConstant = DrawSlider(
                "Angular acceleration response",
                boat.angularAccelerationTimeConstant, 0f, 5f, "s"
            );

            GUILayout.Label("Response is a time constant: larger values respond more slowly; 0 applies commands immediately.", hintStyle);

            boat.linearDamping = DrawSlider(
                "Linear damping",
                boat.linearDamping, 0f, 2f, "1/s"
            );

            boat.angularDamping = DrawSlider(
                "Angular damping",
                boat.angularDamping, 0f, 5f, "1/s"
            );
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
        boat.maxAcceleration = 1.5f;
        boat.maxForwardSpeed = 5f;
        boat.maxReverseSpeed = 1f;

        boat.maxAngularAccelerationDeg = 60f;
        boat.maxYawRateDeg = 45f;

        boat.accelerationTimeConstant = 0f;
        boat.angularAccelerationTimeConstant = 0f;
        boat.speedResponseTime = 0.5f;
        boat.headingResponseTime = 0.5f;

        boat.linearDamping = 0.2f;
        boat.angularDamping = 1.2f;
    }
}
