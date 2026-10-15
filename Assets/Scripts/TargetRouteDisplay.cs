using System;
using System.Collections.Generic;
using System.IO;
using TMPro;
using UnityEngine;

public sealed class TargetRouteDisplay : MonoBehaviour
{
    public TMP_FontAsset fontAsset;
    [Min(0.001f)] public float lineWidth = 0.06f;
    public bool IsLoaded => generatedRoot != null && generatedRoot.activeSelf;
    public string LoadError { get; private set; }
    private GameObject generatedRoot;
    private readonly List<Material> ownedMaterials = new List<Material>();

    void OnEnable() { LoadRoute(); }

    public bool LoadRoute()
    {
        ClearVisuals();
        LoadError = null;
#if UNITY_EDITOR
        string path = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "TestData", "fixed_route.json"));
        try
        {
            var route = TargetRouteData.Parse(File.ReadAllText(path));
            if (fontAsset == null) throw new InvalidOperationException("Assign the existing TMP font asset.");
            if (!BoatDynamics2D.IsFinite(lineWidth) || lineWidth <= 0)
                throw new InvalidOperationException("Route line width must be finite and positive.");
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
                    circle[vertex] = pointPosition + new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0) * 0.18f;
                }
                MakeLine("Waypoint_" + point.id, circle, true, 0.07f, 0, material);
                var labelObject = new GameObject("Label_" + point.id);
                labelObject.transform.SetParent(generatedRoot.transform, false);
                labelObject.transform.position = pointPosition + new Vector3(0, 0.5f, 0);
                var label = labelObject.AddComponent<TextMeshPro>();
                label.font = fontAsset;
                label.fontSharedMaterial = fontAsset.material;
                label.text = point.id;
                label.richText = false;
                label.fontSize = 3.2f;
                label.color = Color.white;
                label.alignment = TextAlignmentOptions.Center;
                label.rectTransform.sizeDelta = new Vector2(3f, 1f);
                label.GetComponent<MeshRenderer>().sortingOrder = 1;
            }
            generatedRoot.SetActive(true);
            return true;
        }
        catch (Exception exception)
        {
            ClearVisuals();
            LoadError = "Could not display route at " + path + ": " + exception.Message;
            Debug.LogError(LoadError, this);
            return false;
        }
#else
        LoadError = "Target route display currently loads project TestData only in Unity Editor; standalone route loading is not supported.";
        Debug.LogWarning(LoadError, this);
        return false;
#endif
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
        foreach (Material material in ownedMaterials) if (material != null) Release(material);
        ownedMaterials.Clear();
    }

    private static void Release(UnityEngine.Object value)
    {
        if (Application.isPlaying) Destroy(value);
        else DestroyImmediate(value);
    }

    void OnDisable() { ClearVisuals(); }
    void OnDestroy() { ClearVisuals(); }
}
