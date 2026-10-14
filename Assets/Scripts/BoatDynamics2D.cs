
using UnityEngine;

public class BoatDynamics2D : MonoBehaviour
{
    [Header("运动限制")]
    public float maxAcceleration = 1.5f;
    public float maxForwardSpeed = 5f;
    public float maxReverseSpeed = 1f;

    public float maxAngularAccelerationDeg = 60f;
    public float maxYawRateDeg = 45f;

    [Header("响应延迟")]
    public float accelerationTimeConstant = 0.25f;
    public float angularAccelerationTimeConstant = 0.25f;

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

    void Start()
    {
        position = transform.position;
        headingRad = transform.eulerAngles.z * Mathf.Deg2Rad;

        startPosition = position;
        startHeadingRad = headingRad;
    }

    void Update()
    {
        // W 前进，S 后退
        accelerationCommand =
            Input.GetAxisRaw("Vertical") * maxAcceleration;

        // A 左转，D 右转
        angularAccelerationCommand =
            -Input.GetAxisRaw("Horizontal")
            * maxAngularAccelerationDeg * Mathf.Deg2Rad;

        // R 重置
        if (Input.GetKeyDown(KeyCode.R))
        {
            ResetBoat();
        }
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // 1. 模拟加速度指令的响应延迟
        actualAcceleration = FirstOrderLag(
            actualAcceleration,
            accelerationCommand,
            dt,
            accelerationTimeConstant
        );

        actualAngularAcceleration = FirstOrderLag(
            actualAngularAcceleration,
            angularAccelerationCommand,
            dt,
            angularAccelerationTimeConstant
        );

        // 2. 积分得到线速度和角速度
        speed +=
            (actualAcceleration - linearDamping * speed) * dt;

        yawRate +=
            (actualAngularAcceleration - angularDamping * yawRate) * dt;

        // 3. 限制速度
        speed = Mathf.Clamp(
            speed, -maxReverseSpeed, maxForwardSpeed
        );

        float maxYawRate =
            maxYawRateDeg * Mathf.Deg2Rad;

        yawRate = Mathf.Clamp(
            yawRate, -maxYawRate, maxYawRate
        );

        // 4. 积分得到航向角
        headingRad += yawRate * dt;

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

    float FirstOrderLag(
        float current,
        float target,
        float dt,
        float timeConstant)
    {
        if (timeConstant <= 0f)
            return target;

        float blend = 1f - Mathf.Exp(-dt / timeConstant);

        return current + (target - current) * blend;
    }

    void ResetBoat()
    {
        position = startPosition;
        headingRad = startHeadingRad;

        speed = 0f;
        yawRate = 0f;

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
    }
}
