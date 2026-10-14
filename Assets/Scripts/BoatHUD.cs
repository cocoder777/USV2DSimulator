
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

        statusText.text =
            "USV 2D SIMULATOR\n\n" +

            $"Position X: {boat.Position.x:F2} m\n" +
            $"Position Y: {boat.Position.y:F2} m\n" +
            $"Speed: {boat.Speed:F2} m/s\n" +
            $"Heading: {boat.HeadingDeg:F1} deg\n" +
            $"Yaw Rate: {boat.YawRateDeg:F1} deg/s\n\n" +

            $"Accel Command: {boat.AccelerationCommandValue:F2} m/s^2\n" +
            $"Actual Accel: {boat.ActualAccelerationValue:F2} m/s^2\n" +
            $"Angular Command: {boat.AngularAccelerationCommandDeg:F1} deg/s^2\n" +
            $"Actual Angular Accel: {boat.ActualAngularAccelerationDeg:F1} deg/s^2\n\n" +

            "W / S : Forward / Reverse\n" +
            "A / D : Turn Left / Right\n" +
            "R : Reset";
    }
}
