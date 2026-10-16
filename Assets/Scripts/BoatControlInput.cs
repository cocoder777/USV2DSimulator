using UnityEngine;

[RequireComponent(typeof(BoatDynamics2D))]
public class BoatControlInput : MonoBehaviour
{
    private BoatDynamics2D boat;
    void Awake() { boat = GetComponent<BoatDynamics2D>(); }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.R)) { boat.ResetBoat(); return; }
        if (boat.Mode == BoatDynamics2D.ControlMode.Manual)
            boat.SetControl(Input.GetAxisRaw("Vertical") * boat.maxAcceleration,
                -Input.GetAxisRaw("Horizontal") * boat.maxAngularAccelerationDeg);
    }

    void OnGUI()
    {
        float width = Mathf.Max(1, Screen.width - 20);
        float modeWidth = width * 0.46f;
        float otherWidth = (width - modeWidth - 12) / 2;
        string label = "Mode: " + boat.Mode;
        if (GUI.Button(new Rect(10, Screen.height - 45, modeWidth, 32), label))
            boat.SetMode((BoatDynamics2D.ControlMode)(((int)boat.Mode + 1) % 2));
        if (GUI.Button(new Rect(16 + modeWidth, Screen.height - 45, otherWidth, 32), "Reload task"))
            GetComponent<MissionManager>()?.Reload();
        var runner = GetComponent<SimulationRunner>();
        if (boat.Mode == BoatDynamics2D.ControlMode.Jev && runner != null
            && GUI.Button(new Rect(22 + modeWidth + otherWidth, Screen.height - 45, otherWidth, 32),
                runner.JevStarted ? "Retry Jev" : "Start Jev"))
        {
            if (runner.JevStarted) runner.Retry(); else runner.StartJev();
        }
    }

    void OnDisable() { if (boat != null) boat.ClearControl(); }
}
