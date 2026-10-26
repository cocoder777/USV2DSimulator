using UnityEngine;

public class BoatCameraFollow : MonoBehaviour
{
    public Transform target;
    public Vector3 offset = new Vector3(0, 0, -10);
    [Min(0.01f)] public float smoothTime = 0.15f;
    [Min(1)] public float orthographicSize = 160f;
    public bool overview;
    private float zoomVelocity, zoomFactor = 1;
    private Camera view;
    private MissionManager mission;
    void Start()
    {
        view = GetComponent<Camera>();
        if (target == null) target = GameObject.Find("Boat")?.transform;
        if (target != null) mission = target.GetComponent<MissionManager>();
    }
    void LateUpdate()
    {
        if (target == null || view == null) return;
        float scale = Mathf.Clamp(Screen.width / 1280f, 0.4f, 1.5f);
        float panelHeight = Mathf.Max(160, Mathf.Min(570, Screen.height / scale - 70));
        var panel = new Rect(Screen.width - 330 * scale, Screen.height - (panelHeight + 10) * scale,
            320 * scale, panelHeight * scale);
        float wheel = panel.Contains(Input.mousePosition) ? 0 : Input.mouseScrollDelta.y;
        if (wheel != 0) zoomFactor = Mathf.Clamp(zoomFactor * Mathf.Pow(0.85f, wheel), 0.4f, 5f);
        Vector3 focus = target.position;
        // Follow the rendered boat exactly, including while waiting for a decision.
        float size = orthographicSize;
        if (overview && mission?.Route != null)
        {
            var points = mission.Route.waypoints;
            var bounds = new Bounds(new Vector3(points[0].x, points[0].y, 0), Vector3.zero);
            foreach (var point in points) bounds.Encapsulate(new Vector3(point.x, point.y, 0));
            focus = bounds.center;
            size = Mathf.Max(bounds.extents.y + 120, (bounds.extents.x + 120) / view.aspect);
        }
        view.orthographicSize = Mathf.SmoothDamp(view.orthographicSize, size * zoomFactor,
            ref zoomVelocity, smoothTime, Mathf.Infinity, Time.unscaledDeltaTime);
        transform.position = focus + offset;
    }
}
