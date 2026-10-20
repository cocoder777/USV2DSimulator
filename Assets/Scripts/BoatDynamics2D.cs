
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

    public void SetMode(ControlMode mode)
    {
        if (Mode == mode) return;
        Mode = mode;
        navigationHeadingTarget = headingRad * Mathf.Rad2Deg;
        headingTransitionRemaining = Mathf.Abs(yawRate) > 0 ? headingResponseTime : 0;
        headingResponseDuration = headingResponseTime;
        navigationSpeedTarget = speed;
        navigationTransitionRemaining = 0;
        navigationResponseDuration = speedResponseTime;
        RunId = System.Guid.NewGuid().ToString("N");
        ClearControl();
        ModeChanged?.Invoke();
    }

    // Main-thread interface. Angular input is degrees/s^2, not radians/s^2.
    public void SetControl(float acceleration, float angularAccelerationDeg)
    {
        if (!IsFinite(acceleration) || !IsFinite(angularAccelerationDeg)) return;
        accelerationCommand = Mathf.Clamp(acceleration, -maxAcceleration, maxAcceleration);
        angularAccelerationCommand = Mathf.Clamp(angularAccelerationDeg,
            -maxAngularAccelerationDeg, maxAngularAccelerationDeg) * Mathf.Deg2Rad;
    }

    public static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    public void ClearControl() { accelerationCommand = 0f; angularAccelerationCommand = 0f; }
    [Header("Jev 航速响应")]
    [Min(0)] public float speedResponseTime = 0.5f;
    [Min(0)] public float headingResponseTime = 0.5f;
    private float navigationHeadingTarget, headingTransitionRemaining, headingResponseDuration;
    public float HeadingTargetOffset => navigationHeadingTarget - headingRad * Mathf.Rad2Deg;
    public float HeadingTransitionRemaining => headingResponseTime == headingResponseDuration
        ? headingTransitionRemaining : headingResponseTime;
    private float navigationSpeedTarget;
    private float navigationTransitionRemaining, navigationResponseDuration;
    public float NavigationTargetSpeed => navigationSpeedTarget;
    public float SpeedTransitionRemaining => speedResponseTime == navigationResponseDuration
        ? navigationTransitionRemaining : speedResponseTime;

    // Speed ramps linearly; heading uses a cubic with continuous yaw and zero yaw at arrival.
    public void SetNavigationTarget(float targetSpeed, float targetHeadingDeg)
    {
        if (!IsFinite(targetSpeed) || !IsFinite(targetHeadingDeg)
            || !IsFinite(speedResponseTime) || speedResponseTime < 0
            || !IsFinite(headingResponseTime) || headingResponseTime < 0)
            throw new System.ArgumentException("Navigation target must be finite.");
        targetSpeed = Mathf.Clamp(targetSpeed, -maxReverseSpeed, maxForwardSpeed);
        if (targetSpeed != navigationSpeedTarget || speedResponseTime != navigationResponseDuration)
        {
            navigationSpeedTarget = targetSpeed;
            navigationTransitionRemaining = speedResponseTime;
            navigationResponseDuration = speedResponseTime;
        }
        if (Mathf.Abs(Mathf.DeltaAngle(navigationHeadingTarget, targetHeadingDeg)) > 0.0001f
            || headingResponseDuration != headingResponseTime)
        {
            navigationHeadingTarget = headingRad * Mathf.Rad2Deg + Mathf.DeltaAngle(HeadingDeg, targetHeadingDeg);
            headingTransitionRemaining = headingResponseTime;
            headingResponseDuration = headingResponseTime;
        }
        ClearControl();
    }
    [Header("运动限制")]
    public float maxAcceleration = 1.5f;
    public float maxForwardSpeed = 5f;
    public float maxReverseSpeed = 1f;

    public float maxAngularAccelerationDeg = 60f;
    public float maxYawRateDeg = 45f;

    [Header("响应延迟")]
    public float accelerationTimeConstant = 0f;
    public float angularAccelerationTimeConstant = 0f;

    [Header("阻尼")]
    public float linearDamping = 0.2f;
    public float angularDamping = 1.2f;

    // 运动状态
    private Vector2 position;
    private float speed;
    private float headingRad;
    private float yawRate;

    // 对外提供只读状态，供 UI 和其他模块使用
    public Vector2 Position => position;

    public float Speed => speed;

    public float HeadingDeg =>
        Mathf.Repeat(headingRad * Mathf.Rad2Deg, 360f);

    public float YawRateDeg =>
        yawRate * Mathf.Rad2Deg;

    public float AccelerationCommandValue =>
        accelerationCommand;

    public float ActualAccelerationValue =>
        actualAcceleration;

    public float AngularAccelerationCommandDeg =>
        angularAccelerationCommand * Mathf.Rad2Deg;

    public float ActualAngularAccelerationDeg =>
        actualAngularAcceleration * Mathf.Rad2Deg;
    // 实际产生的加速度
    private float actualAcceleration;
    private float actualAngularAcceleration;

    // 外部控制指令
    private float accelerationCommand;
    private float angularAccelerationCommand;

    // 用于重置
    private Vector2 startPosition;
    private float startHeadingRad;
    private float startSpeed, startYawRate;

    public void ConfigureInitialState(TargetRouteData.InitialState initial)
    {
        startPosition = new Vector2(initial.x, initial.y);
        startHeadingRad = initial.heading_deg * Mathf.Deg2Rad;
        startSpeed = initial.speed;
        startYawRate = initial.yaw_rate_deg * Mathf.Deg2Rad;
    }

    void Awake()
    {
        position = transform.position;
        headingRad = transform.eulerAngles.z * Mathf.Deg2Rad;

        startPosition = position;
        startHeadingRad = headingRad;
    }

    public void Step(float dt)
    {
        if (!IsFinite(dt) || dt <= 0) throw new System.ArgumentOutOfRangeException(nameof(dt));
        Tick++;
        SimulationTime += dt;
        // Parameter changes take effect on an already held command too.
        SetControl(accelerationCommand, angularAccelerationCommand * Mathf.Rad2Deg);

        Vector2 displacement;
        if (Mode == ControlMode.Manual)
        {
            (speed, actualAcceleration) = BoatScalarDynamics.Step(speed, actualAcceleration, accelerationCommand,
                accelerationTimeConstant, linearDamping, dt, -maxReverseSpeed, maxForwardSpeed);
            (yawRate, actualAngularAcceleration) = BoatScalarDynamics.Step(yawRate, actualAngularAcceleration,
                angularAccelerationCommand, angularAccelerationTimeConstant, angularDamping, dt,
                -maxYawRateDeg * Mathf.Deg2Rad, maxYawRateDeg * Mathf.Deg2Rad);

            // 4. 积分得到航向角
            headingRad += yawRate * dt;
            displacement = new Vector2(Mathf.Cos(headingRad), Mathf.Sin(headingRad)) * speed * dt;
        }
        else
        {
            float oldSpeed = Mathf.Clamp(speed, -maxReverseSpeed, maxForwardSpeed);
            float targetSpeed = Mathf.Clamp(navigationSpeedTarget, -maxReverseSpeed, maxForwardSpeed);
            float remaining = navigationTransitionRemaining;
            float turnRemaining = headingTransitionRemaining;
            float oldYaw = yawRate;
            float initialHeading = headingRad * Mathf.Rad2Deg;
            float delta = navigationHeadingTarget - initialHeading;
            // Simpson integration at the same five samples used by Python's option descriptions.
            Vector2 Velocity(float t)
            {
                float v = NavigationSpeed(oldSpeed, targetSpeed, remaining, t);
                var turn = NavigationTurn(delta, oldYaw * Mathf.Rad2Deg, turnRemaining, t);
                float angle = (initialHeading + turn.angle) * Mathf.Deg2Rad;
                return new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)) * v;
            }
            displacement = Vector2.zero;
            float[] boundaries = { 0, dt, Mathf.Min(dt, remaining), Mathf.Min(dt, turnRemaining) };
            System.Array.Sort(boundaries);
            for (int i = 1; i < boundaries.Length; i++)
            {
                float start = boundaries[i - 1], span = boundaries[i] - start;
                if (span <= 0) continue;
                displacement += (Velocity(start) + 4 * Velocity(start + span / 4)
                    + 2 * Velocity(start + span / 2) + 4 * Velocity(start + 3 * span / 4)
                    + Velocity(start + span)) * (span / 12);
            }
            speed = NavigationSpeed(oldSpeed, targetSpeed, remaining, dt);
            var endTurn = NavigationTurn(delta, oldYaw * Mathf.Rad2Deg, turnRemaining, dt);
            headingRad = (initialHeading + endTurn.angle) * Mathf.Deg2Rad;
            yawRate = endTurn.yaw * Mathf.Deg2Rad;
            navigationTransitionRemaining = Mathf.Max(0, remaining - dt);
            headingTransitionRemaining = Mathf.Max(0, turnRemaining - dt);
            if (navigationTransitionRemaining < 0.000001f) navigationTransitionRemaining = 0;
            if (headingTransitionRemaining < 0.000001f) headingTransitionRemaining = 0;
            actualAcceleration = (speed - oldSpeed) / dt;
            actualAngularAcceleration = (yawRate - oldYaw) / dt;
        }

        position += displacement;

        // 6. 更新 Unity 中的船体
        transform.position = new Vector3(
            position.x, position.y, 0f
        );

        transform.rotation = Quaternion.Euler(
            0f, 0f, headingRad * Mathf.Rad2Deg
        );
    }

    public static float NavigationSpeed(float initial, float target, float remaining, float t)
    {
        return remaining <= 0 || t >= remaining ? target : initial + (target - initial) * (t / remaining);
    }

    // Cubic Hermite: start at the measured heading/yaw, arrive at target with zero yaw.
    // A restricted tail of the same cubic is reconstructed each step, so continuing does not reset it.
    public static (float angle, float yaw) NavigationTurn(float delta, float initialYaw, float remaining, float t)
    {
        if (remaining <= 0 || t >= remaining) return (delta, 0);
        float u = t / remaining;
        float angle = (-2*u*u*u + 3*u*u)*delta + (u*u*u - 2*u*u + u)*remaining*initialYaw;
        float yaw = ((-6*u*u + 6*u)*delta + (3*u*u - 4*u + 1)*remaining*initialYaw) / remaining;
        return (angle, yaw);
    }

    public void ResetBoat()
    {
        RunId = System.Guid.NewGuid().ToString("N");
        EpisodeId = System.Guid.NewGuid().ToString("N");
        Tick = 0;
        SimulationTime = 0;
        position = startPosition;
        headingRad = startHeadingRad;

        speed = startSpeed;
        navigationHeadingTarget = headingRad * Mathf.Rad2Deg;
        headingTransitionRemaining = Mathf.Abs(startYawRate) > 0 ? headingResponseTime : 0;
        headingResponseDuration = headingResponseTime;
        navigationSpeedTarget = startSpeed;
        navigationTransitionRemaining = 0;
        navigationResponseDuration = speedResponseTime;
        yawRate = startYawRate;

        actualAcceleration = 0f;
        actualAngularAcceleration = 0f;

        accelerationCommand = 0f;
        angularAccelerationCommand = 0f;

        transform.position = new Vector3(
            position.x, position.y, 0f
        );

        transform.rotation = Quaternion.Euler(
            0f, 0f, headingRad * Mathf.Rad2Deg
        );

        BoatTrail trail = GetComponent<BoatTrail>();
        if (trail != null)
        trail.ClearTrail();
        ResetPerformed?.Invoke();
    }
}
