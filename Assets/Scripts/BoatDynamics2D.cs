using UnityEngine;

public class BoatDynamics2D : MonoBehaviour
{
    public enum ControlMode { Manual, Jev }
    public ControlMode Mode { get; private set; } = ControlMode.Manual;
    public string SessionId { get; } = System.Guid.NewGuid().ToString("N");
    public string RunId { get; private set; } = System.Guid.NewGuid().ToString("N");
    public string EpisodeId { get; private set; } = System.Guid.NewGuid().ToString("N");
    public long Tick { get; private set; }
    public double SimulationTime { get; private set; }
    public event System.Action ResetPerformed;
    public event System.Action ModeChanged;

    [Header("Motion limits (m/s, m/s虏, degrees)")]
    [Min(0.01f)] public float maxForwardSpeed = BoatParameters.DefaultMaxSpeedKnots * DiscreteAction.MetresPerSecondPerKnot;
    [Min(0)] public float maxReverseSpeed = 1f;
    [Min(0.01f)] public float maxAcceleration = BoatParameters.DefaultAcceleration;
    [Min(0.01f)] public float maxDeceleration = BoatParameters.DefaultDeceleration;
    [Min(0.01f)] public float maxAngularAccelerationDeg = BoatParameters.DefaultAngularAcceleration;
    [Min(0.01f)] public float maxYawRateDeg = BoatParameters.DefaultYawRate;
    [Header("Hull dimensions (metres)")]
    [Min(0.01f)] public float lengthMetres = BoatParameters.DefaultLength;
    [Min(0.01f)] public float beamMetres = BoatParameters.DefaultBeam;
    [Header("Waypoint acceptance")]
    [Tooltip("Acceptance radius = circular hull radius times this multiplier. Collision radius is unchanged.")]
    [Min(1)] public float waypointRadiusMultiplier = BoatParameters.DefaultWaypointRadiusMultiplier;
    public float WaypointRadius => CollisionRadius * waypointRadiusMultiplier;
    public float CollisionRadius => Mathf.Sqrt(lengthMetres * lengthMetres + beamMetres * beamMetres) / 2;

    private Vector2 position, startPosition;
    private float speed, headingRad, yawRate, startHeadingRad, startSpeed, startYawRate;
    private float navigationSpeedTarget, navigationHeadingTarget;
    private float accelerationCommand, angularAccelerationCommand, actualAcceleration, actualAngularAcceleration;
    public Vector2 Position => position;
    public float Speed => speed;
    // Mathf.Repeat can round tiny negative angles up to 360 in float precision.
    public static float NormalizeDegrees(float value)
    {
        float result = Mathf.Repeat(value, 360f);
        return result >= 360f ? 0f : result;
    }
    public float HeadingDeg => NormalizeDegrees(headingRad * Mathf.Rad2Deg);
    public float YawRateDeg => yawRate * Mathf.Rad2Deg;
    public float NavigationTargetCourseDeg => NormalizeDegrees(90f - navigationHeadingTarget);
    public float NavigationCourseErrorDeg => -(navigationHeadingTarget - headingRad * Mathf.Rad2Deg);
    public float AccelerationCommandValue => accelerationCommand;
    public float ActualAccelerationValue => actualAcceleration;
    public float AngularAccelerationCommandDeg => angularAccelerationCommand * Mathf.Rad2Deg;
    public float ActualAngularAccelerationDeg => actualAngularAcceleration * Mathf.Rad2Deg;
    public static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);

    public void SetMode(ControlMode mode)
    {
        if (Mode == mode) return;
        Mode = mode;
        navigationSpeedTarget = speed; navigationHeadingTarget = headingRad * Mathf.Rad2Deg;
        RunId = System.Guid.NewGuid().ToString("N"); ClearControl(); ModeChanged?.Invoke();
    }
    public void SetControl(float acceleration, float angularAccelerationDeg)
    {
        if (!IsFinite(acceleration) || !IsFinite(angularAccelerationDeg)) return;
        accelerationCommand = Mathf.Clamp(acceleration, -maxDeceleration, maxAcceleration);
        angularAccelerationCommand = Mathf.Clamp(angularAccelerationDeg,
            -maxAngularAccelerationDeg, maxAngularAccelerationDeg) * Mathf.Deg2Rad;
    }
    public void ClearControl() { accelerationCommand = 0; angularAccelerationCommand = 0; }
    public void SetNavigationTarget(float targetSpeed, float targetHeadingDeg)
    {
        if (!IsFinite(targetSpeed) || !IsFinite(targetHeadingDeg))
            throw new System.ArgumentException("Navigation target must be finite.");
        navigationSpeedTarget = Mathf.Clamp(targetSpeed, -maxReverseSpeed, maxForwardSpeed);
        if (Mathf.Abs(Mathf.DeltaAngle(navigationHeadingTarget, targetHeadingDeg)) > 0.0001f)
            navigationHeadingTarget = headingRad * Mathf.Rad2Deg + Mathf.DeltaAngle(HeadingDeg, targetHeadingDeg);
        ClearControl();
    }
    public void ConfigureInitialState(TargetRouteData.InitialState initial)
    {
        startPosition = new Vector2(initial.x, initial.y); startHeadingRad = initial.heading_deg * Mathf.Deg2Rad;
        startSpeed = initial.speed; startYawRate = initial.yaw_rate_deg * Mathf.Deg2Rad;
    }
    void Awake()
    {
        position = startPosition = transform.position;
        headingRad = startHeadingRad = transform.eulerAngles.z * Mathf.Deg2Rad;
    }
    public void Step(float dt)
    {
        if (!IsFinite(dt) || dt <= 0) throw new System.ArgumentOutOfRangeException(nameof(dt));
        foreach (float limit in new[] { maxForwardSpeed, maxAcceleration, maxDeceleration,
            maxAngularAccelerationDeg, maxYawRateDeg, lengthMetres, beamMetres, waypointRadiusMultiplier })
            if (!IsFinite(limit) || limit <= 0) throw new System.ArgumentException("Motion limits and hull dimensions must be positive.");
        if (!IsFinite(maxReverseSpeed) || maxReverseSpeed < 0) throw new System.ArgumentException("Invalid reverse limit.");
        Tick++; SimulationTime += dt;
        float initialSpeed = speed, initialHeading = headingRad * Mathf.Rad2Deg, initialYaw = YawRateDeg;
        float target = Mathf.Clamp(navigationSpeedTarget, -maxReverseSpeed, maxForwardSpeed);
        float angular = Mathf.Clamp(AngularAccelerationCommandDeg, -maxAngularAccelerationDeg, maxAngularAccelerationDeg);
        (float angle, float yaw) ManualTurn(float t)
        {
            float endYaw = Mathf.Clamp(initialYaw + angular * t, -maxYawRateDeg, maxYawRateDeg);
            float ramp = angular == 0 ? 0 : Mathf.Clamp((endYaw - initialYaw) / angular, 0, t);
            return (initialYaw * ramp + angular * ramp * ramp / 2 + endYaw * (t - ramp), endYaw);
        }
        (float angle, float yaw) Turn(float t) => Mode == ControlMode.Jev
            ? BoatMotion.Turn(navigationHeadingTarget - initialHeading, initialYaw, maxYawRateDeg, maxAngularAccelerationDeg, t)
            : ManualTurn(t);
        float SpeedAt(float t) => Mode == ControlMode.Jev
            ? BoatMotion.Speed(initialSpeed, target, maxAcceleration, maxDeceleration, t)
            : Mathf.Clamp(initialSpeed + Mathf.Clamp(accelerationCommand, -maxDeceleration, maxAcceleration) * t,
                -maxReverseSpeed, maxForwardSpeed);
        Vector2 Velocity(float t)
        {
            float radians = (initialHeading + Turn(t).angle) * Mathf.Deg2Rad;
            return new Vector2(Mathf.Cos(radians), Mathf.Sin(radians)) * SpeedAt(t);
        }
        position += (Velocity(0) + 4 * Velocity(dt / 4) + 2 * Velocity(dt / 2)
            + 4 * Velocity(3 * dt / 4) + Velocity(dt)) * (dt / 12);
        speed = SpeedAt(dt);
        var turn = Turn(dt);
        headingRad = (initialHeading + turn.angle) * Mathf.Deg2Rad; yawRate = turn.yaw * Mathf.Deg2Rad;
        actualAcceleration = (speed - initialSpeed) / dt;
        actualAngularAcceleration = (yawRate - initialYaw * Mathf.Deg2Rad) / dt;
        transform.SetPositionAndRotation(new Vector3(position.x, position.y, 0), Quaternion.Euler(0, 0, HeadingDeg));
    }
    public void ResetBoat()
    {
        RunId = System.Guid.NewGuid().ToString("N"); EpisodeId = System.Guid.NewGuid().ToString("N");
        Tick = 0; SimulationTime = 0; position = startPosition; headingRad = startHeadingRad;
        speed = startSpeed; yawRate = startYawRate;
        navigationHeadingTarget = headingRad * Mathf.Rad2Deg; navigationSpeedTarget = speed;
        actualAcceleration = actualAngularAcceleration = 0; ClearControl();
        transform.SetPositionAndRotation(new Vector3(position.x, position.y, 0), Quaternion.Euler(0, 0, HeadingDeg));
        GetComponent<BoatTrail>()?.ClearTrail(); ResetPerformed?.Invoke();
    }
}
