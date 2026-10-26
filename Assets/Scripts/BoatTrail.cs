
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class BoatTrail : MonoBehaviour
{
    public float pointSpacing = 2f;
    public int maxPoints = 12000;
    public float lineWidth = 0.7f;

    private LineRenderer line;
    private readonly List<Vector3> points = new List<Vector3>();
    private Vector3 lastPoint;

    void Awake()
    {
        line = GetComponent<LineRenderer>();

        line.useWorldSpace = true;
        line.positionCount = 0;
        line.startWidth = lineWidth;
        line.endWidth = lineWidth;
        line.numCapVertices = 4;
        line.numCornerVertices = 4;
        line.sortingOrder = -1;

        Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
        if (shader == null)
            shader = Shader.Find("Sprites/Default");

        if (shader != null)
            line.material = new Material(shader);

        line.startColor = Color.cyan;
        line.endColor = Color.cyan;
    }

    void Start()
    {
        AddPoint(transform.position);
    }

    void LateUpdate()
    {
        Vector3 current = transform.position;

        if (points.Count == 0 ||
            Vector3.Distance(current, lastPoint) >= pointSpacing)
        {
            AddPoint(current);
        }
    }

    void AddPoint(Vector3 point)
    {
        points.Add(point);
        lastPoint = point;

        if (points.Count > maxPoints)
            points.RemoveAt(0);

        line.positionCount = points.Count;
        line.SetPositions(points.ToArray());
    }

    public void ClearTrail()
    {
        points.Clear();
        line.positionCount = 0;
        lastPoint = transform.position;
        AddPoint(lastPoint);
    }
}
