
using TMPro;
using UnityEngine;

public class BoatHUD : MonoBehaviour
{
    public BoatDynamics2D boat;
    public TextMeshProUGUI statusText;

    void Update()
    {
        if (boat == null || statusText == null)
            return;

        BoatUdpBridge bridge = boat.GetComponent<BoatUdpBridge>();
        string connection = bridge == null ? "Unavailable" : bridge.CommunicationStatus;
        var mission = boat.GetComponent<MissionManager>();
        var decision = boat.GetComponent<SimulationRunner>();
        var model = boat.GetComponent<LocalModelSwitcher>();
        statusText.text =
            "USV 2D SIMULATOR\n" +
            $"Mode: {boat.Mode}\nUDP: {connection}\n" +
            $"Model: {model?.ActiveModelName ?? "external / not selected"}\n" +
            $"Task: {mission?.Status ?? "NoTask"}  Next: {mission?.Progress?.Next?.id ?? "--"}\n" +
            $"Decision: {decision?.Status ?? "--"}\n" +
            $"Action duration: {decision?.actionDuration:F2} s\n" +

            $"Position X: {boat.Position.x:F2} m\n" +
            $"Position Y: {boat.Position.y:F2} m\n" +
            $"Speed: {boat.Speed:F2} m/s\n" +
            $"Heading: {boat.HeadingDeg:F1} deg\n" +
            $"Yaw Rate: {boat.YawRateDeg:F1} deg/s\n" +

            $"Accel Command: {boat.AccelerationCommandValue:F2} m/s^2\n" +
            $"Actual Accel: {boat.ActualAccelerationValue:F2} m/s^2\n" +
            $"Angular Command: {boat.AngularAccelerationCommandDeg:F1} deg/s^2\n" +
            $"Actual Angular Accel: {boat.ActualAngularAccelerationDeg:F1} deg/s^2\n" +

            "W / S : Forward / Reverse\n" +
            "A / D : Turn Left / Right\n" +
            "R : Reset";
    }
}
