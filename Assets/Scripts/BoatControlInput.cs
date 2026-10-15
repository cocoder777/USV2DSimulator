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
        string label = boat.Mode == BoatDynamics2D.ControlMode.Manual
            ? "Mode: Manual - switch to External" : "Mode: External - switch to Manual";
        if (GUI.Button(new Rect(10, Screen.height - 45, 290, 32), label))
            boat.SetMode(boat.Mode == BoatDynamics2D.ControlMode.Manual
                ? BoatDynamics2D.ControlMode.External : BoatDynamics2D.ControlMode.Manual);
    }

    void OnDisable() { if (boat != null) boat.ClearControl(); }
}
