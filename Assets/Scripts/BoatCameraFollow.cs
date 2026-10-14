
using UnityEngine;

public class BoatCameraFollow : MonoBehaviour
{
    [Header("跟随目标")]
    public Transform target;

    [Header("摄像机设置")]
    public Vector3 offset = new Vector3(0f, 0f, -10f);
    public float smoothTime = 0.15f;

    private Vector3 currentVelocity;

    void Start()
    {
        // 如果没有手动指定目标，自动寻找名为 Boat 的对象
        if (target == null)
        {
            GameObject boat = GameObject.Find("Boat");

            if (boat != null)
                target = boat.transform;
        }
    }

    void LateUpdate()
    {
        if (target == null)
            return;

        // 跟随船的位置，但保持摄像机在船后方
        Vector3 targetPosition = target.position + offset;

        transform.position = Vector3.SmoothDamp(
            transform.position,
            targetPosition,
            ref currentVelocity,
            smoothTime
        );
    }
}
