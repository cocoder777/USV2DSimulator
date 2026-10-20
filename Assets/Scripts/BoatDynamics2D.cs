
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
    // Jev navigation targets are instantaneous in v1; manual acceleration dynamics remain separate.
    public void SetNavigationTarget(float targetSpeed, float targetHeadingDeg)
    {
        if (!IsFinite(targetSpeed) || !IsFinite(targetHeadingDeg))
            throw new System.ArgumentException("Navigation target must be finite.");
        speed = Mathf.Clamp(targetSpeed, -maxReverseSpeed, maxForwardSpeed);
        headingRad = targetHeadingDeg * Mathf.Deg2Rad;
        yawRate = 0;
        actualAcceleration = actualAngularAcceleration = 0;
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

        if (Mode == ControlMode.Manual)
        {
            (speed, actualAcceleration) = BoatScalarDynamics.Step(speed, actualAcceleration, accelerationCommand,
                accelerationTimeConstant, linearDamping, dt, -maxReverseSpeed, maxForwardSpeed);
            (yawRate, actualAngularAcceleration) = BoatScalarDynamics.Step(yawRate, actualAngularAcceleration,
                angularAccelerationCommand, angularAccelerationTimeConstant, angularDamping, dt,
                -maxYawRateDeg * Mathf.Deg2Rad, maxYawRateDeg * Mathf.Deg2Rad);

            // 4. 积分得到航向角
            headingRad += yawRate * dt;
        }

        // 5. 根据船头方向更新二维位置
        float dx = Mathf.Cos(headingRad) * speed * dt;
        float dy = Mathf.Sin(headingRad) * speed * dt;

        position += new Vector2(dx, dy);

        // 6. 更新 Unity 中的船体
        transform.position = new Vector3(
            position.x, position.y, 0f
        );

        transform.rotation = Quaternion.Euler(
            0f, 0f, headingRad * Mathf.Rad2Deg
        );
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
