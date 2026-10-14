
using UnityEngine;

public class BoatParameterPanel : MonoBehaviour
{
    private BoatDynamics2D boat;

    private GUIStyle titleStyle;
    private GUIStyle labelStyle;
    private GUIStyle hintStyle;

    private Rect panelRect;

    void Awake()
    {
        boat = GetComponent<BoatDynamics2D>();
    }

    void OnGUI()
    {
        if (boat == null)
            return;

        // 面板固定在屏幕右上角
        panelRect = new Rect(
            Mathf.Max(10f, Screen.width - 330f),
            10f,
            320f,
            570f
        );

        GUI.Box(panelRect, GUIContent.none);

        GUILayout.BeginArea(new Rect(
            panelRect.x + 12f,
            panelRect.y + 10f,
            panelRect.width - 24f,
            panelRect.height - 20f
        ));

        InitializeStyles();

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

        boat.maxAcceleration = DrawSlider(
            "Max acceleration",
            boat.maxAcceleration, 0.1f, 5f, "m/s^2"
        );

        boat.accelerationTimeConstant = DrawSlider(
            "Acceleration response",
            boat.accelerationTimeConstant, 0f, 2f, "s"
        );

        boat.maxAngularAccelerationDeg = DrawSlider(
            "Max angular acceleration",
            boat.maxAngularAccelerationDeg, 5f, 180f, "deg/s^2"
        );

        boat.angularAccelerationTimeConstant = DrawSlider(
            "Angular response",
            boat.angularAccelerationTimeConstant, 0f, 2f, "s"
        );

        boat.maxYawRateDeg = DrawSlider(
            "Max turning rate",
            boat.maxYawRateDeg, 5f, 120f, "deg/s"
        );

        boat.linearDamping = DrawSlider(
            "Linear damping",
            boat.linearDamping, 0f, 2f, "1/s"
        );

        boat.angularDamping = DrawSlider(
            "Angular damping",
            boat.angularDamping, 0f, 5f, "1/s"
        );

        GUILayout.Space(6);

        if (GUILayout.Button(
            "Restore Default Parameters",
            GUILayout.Height(28)
        ))
        {
            RestoreDefaults();
        }

        GUILayout.Space(4);
        GUILayout.Label(
            "Response = 0 means no input filtering.",
            hintStyle
        );
        GUILayout.Label(
            "W/S: accelerate   A/D: turn   R: reset boat",
            hintStyle
        );

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

        boat.accelerationTimeConstant = 0.25f;
        boat.angularAccelerationTimeConstant = 0.25f;

        boat.linearDamping = 0.2f;
        boat.angularDamping = 1.2f;
    }
}
