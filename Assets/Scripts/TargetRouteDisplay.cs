using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public sealed class TargetRouteDisplay : MonoBehaviour
{
    public TMP_FontAsset fontAsset;
    [Min(0.001f)] public float lineWidth = 1f;
    public bool IsLoaded => generatedRoot != null && generatedRoot.activeSelf;
    public string LoadError { get; private set; }
    private GameObject generatedRoot;
    private readonly List<Material> ownedMaterials = new List<Material>();
    private MissionManager mission;
    private int displayedProgress = -1;
    private float displayedRadius;
    private readonly List<LineRenderer> markers = new List<LineRenderer>();
    private readonly List<TextMeshPro> labels = new List<TextMeshPro>();

    void OnEnable()
    {
        mission = FindAnyObjectByType<MissionManager>();
        if (mission != null) mission.RouteChanged += ReloadFromMission;
        if (mission?.Route != null) LoadRoute();
    }

    private void ReloadFromMission() { LoadRoute(); }

    void LateUpdate()
    {
        if (mission?.Progress == null) return;
        if (!Mathf.Approximately(displayedRadius, mission.Route.arrival_radius_m)) LoadRoute();
        for (int i = 0; i < labels.Count; i++)
        {
            var point = mission.Route.waypoints[i];
            float? peak = mission.Progress.PassingMaxSpeeds[i];
            float visualScale = Camera.main == null ? 1 : Mathf.Max(1, Camera.main.orthographicSize / 160);
            labels[i].fontSize = 16 * visualScale;
            labels[i].rectTransform.sizeDelta = new Vector2(100, 40) * visualScale;
            labels[i].transform.position = new Vector3(point.x, point.y + mission.Route.arrival_radius_m + 14 * visualScale, 0.01f);
            string text = point.id + "\n计划航速（通过上限）：" + point.routeSpeed.ToString("F2") + " m/s";
            if (point.role == "start") { labels[i].text = point.id; continue; }
            if (peak.HasValue) text += "\n实际通过最高航速：" + peak.Value.ToString("F2") + " m/s"
                + (peak.Value > point.routeSpeed + 0.001f ? "（超限）" : "");
            if (labels[i].text != text) labels[i].text = text;
        }
        if (displayedProgress == mission.Progress.NextIndex) return;
        displayedProgress = mission.Progress.NextIndex;
        for (int i = 0; i < markers.Count; i++)
        {
            string role = mission.Route.waypoints[i].role;
            Color roleColor = role == "start" ? Color.green : role == "end" ? Color.red : new Color(1, 0.5f, 0);
            markers[i].sharedMaterial.color = i < displayedProgress ? Color.cyan : i == displayedProgress ? Color.white : roleColor;
        }
    }

    public bool LoadRoute()
    {
        LoadError = null;
        try
        {
            if (mission == null) mission = FindAnyObjectByType<MissionManager>();
            var route = mission?.Route;
            if (route != null) displayedRadius = route.arrival_radius_m;
            if (route == null) throw new InvalidOperationException("MissionManager has not loaded a task.");
            if (fontAsset == null) throw new InvalidOperationException("Assign the existing TMP font asset.");
            if (!BoatDynamics2D.IsFinite(lineWidth) || lineWidth <= 0)
                throw new InvalidOperationException("Route line width must be finite and positive.");
            ClearVisuals();
            // Validate everything before exposing any visuals.
            generatedRoot = new GameObject("TargetRouteVisuals");
            generatedRoot.SetActive(false);
            generatedRoot.transform.SetParent(transform, false);
            Shader shader = Shader.Find("Sprites/Default");
            if (shader == null) shader = Shader.Find("Universal Render Pipeline/2D/Sprite-Unlit-Default");
            if (shader == null) throw new InvalidOperationException("No supported transparent sprite shader found.");
            Material routeMaterial = CreateMaterial(shader, Color.yellow);
            Material startMaterial = CreateMaterial(shader, Color.green);
            Material checkpointMaterial = CreateMaterial(shader, new Color(1f, 0.5f, 0f));
            Material endMaterial = CreateMaterial(shader, Color.red);
            var reference = route.GetReferenceSamples();
            var positions = new Vector3[reference.Length];
            for (int i = 0; i < positions.Length; i++)
                // Slightly behind the boat/trail so an opaque trail material also stays visible.
                positions[i] = new Vector3(reference[i].x, reference[i].y, 0.01f);
            MakeLine("TargetPath", positions, false, lineWidth, -2, routeMaterial);
            for (int i = 0; i < route.waypoints.Length; i++)
            {
                var point = route.waypoints[i];
                Vector3 pointPosition = new Vector3(point.x, point.y, 0.01f);
                Material material = point.role == "start" ? startMaterial
                    : point.role == "end" ? endMaterial : checkpointMaterial;
                var circle = new Vector3[32];
                for (int vertex = 0; vertex < circle.Length; vertex++)
                {
                    float angle = vertex * Mathf.PI * 2f / circle.Length;
                    circle[vertex] = pointPosition + new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0) * route.arrival_radius_m;
                }
                MakeLine("Waypoint_" + point.id, circle, true, 0.45f, 0, material);
                var marker = generatedRoot.transform.Find("Waypoint_" + point.id).GetComponent<LineRenderer>();
                // Give each marker its own material so completed colors do not affect all checkpoints.
                marker.sharedMaterial = CreateMaterial(shader, material.color);
                markers.Add(marker);
                var labelObject = new GameObject("Label_" + point.id);
                labelObject.transform.SetParent(generatedRoot.transform, false);
                labelObject.transform.position = pointPosition + new Vector3(0, route.arrival_radius_m + 14f, 0);
                var label = labelObject.AddComponent<TextMeshPro>();
                label.font = fontAsset;
                var cjk = Resources.Load<TMP_FontAsset>("WaypointChinese");
                if (cjk != null) label.font = cjk;
                label.fontSharedMaterial = label.font.material;
                label.text = point.id;
                label.richText = false;
                label.fontSize = 16f;
                label.color = Color.white;
                label.alignment = TextAlignmentOptions.Center;
                label.rectTransform.sizeDelta = new Vector2(100f, 40f);
                label.GetComponent<MeshRenderer>().sortingOrder = 1;
                labels.Add(label);
            }
            generatedRoot.SetActive(true);
            return true;
        }
        catch (Exception exception)
        {
            ClearVisuals();
            LoadError = "Could not display the active mission: " + exception.Message;
            Debug.LogError(LoadError, this);
            return false;
        }
    }

    private Material CreateMaterial(Shader shader, Color color)
    {
        var material = new Material(shader) { name = "TargetRoute material", color = color };
        ownedMaterials.Add(material);
        material.renderQueue = 3000;
        return material;
    }

    private void MakeLine(string objectName, Vector3[] positions, bool loop, float width,
        int sortingOrder, Material material)
    {
        var lineObject = new GameObject(objectName);
        lineObject.transform.SetParent(generatedRoot.transform, false);
        var line = lineObject.AddComponent<LineRenderer>();
        line.useWorldSpace = true;
        line.sharedMaterial = material;
        line.startColor = line.endColor = Color.white;
        line.startWidth = line.endWidth = width;
        line.numCapVertices = line.numCornerVertices = 4;
        line.sortingOrder = sortingOrder;
        line.loop = loop;
        line.positionCount = positions.Length;
        line.SetPositions(positions);
    }

    private void ClearVisuals()
    {
        if (generatedRoot != null) { generatedRoot.SetActive(false); Release(generatedRoot); }
        generatedRoot = null;
        markers.Clear();
        labels.Clear();
        displayedProgress = -1;
        foreach (Material material in ownedMaterials) if (material != null) Release(material);
        ownedMaterials.Clear();
    }

    private static void Release(UnityEngine.Object value)
    {
        if (Application.isPlaying) Destroy(value);
        else DestroyImmediate(value);
    }

    void OnDisable() { if (mission != null) mission.RouteChanged -= ReloadFromMission; ClearVisuals(); }
    void OnDestroy() { ClearVisuals(); }
}
