using System;
using System.IO;
using System.Reflection;
using TMPro;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class TargetRouteValidation
{
    private static void Check(bool condition, string message)
    {
        if (!condition) throw new Exception(message);
    }

    public static void Run()
    {
        GameObject temporaryBoat = null;
        TargetRouteDisplay display = null;
        try
        {
            EditorSceneManager.OpenScene("Assets/Scenes/Main.unity");
            var root = GameObject.Find("TargetRoute");
            Check(root != null && root.transform.parent == null, "Route scene root missing");
            display = root.GetComponent<TargetRouteDisplay>();
            Check(display != null && display.fontAsset != null && display.LoadRoute(), "Route did not load");
            var route = TargetRouteData.Parse(File.ReadAllText(Path.Combine(Application.dataPath,
                "..", "TestData", "fixed_route.json")));
            var visuals = root.transform.Find("TargetRouteVisuals");
            var line = visuals.Find("TargetPath").GetComponent<LineRenderer>();
            var samples = route.GetReferenceSamples();
            Check(route.schema_version == 2 && line.positionCount == 401 && samples.Length == 401
                && line.useWorldSpace && line.sortingOrder == -2
                && Mathf.Approximately(line.startWidth, 0.06f)
                && line.sharedMaterial.color == Color.yellow, "Target line configuration incorrect");
            Check(visuals.GetComponentsInChildren<LineRenderer>().Length == 6, "Expected path and five rings");
            Check(visuals.GetComponentsInChildren<TextMeshPro>().Length == 5, "Expected five labels");
            for (int i = 0; i < samples.Length; i++)
            {
                Vector3 position = line.GetPosition(i);
                Check(position.x == samples[i].x && position.y == samples[i].y, "Curve sample mismatch");
            }
            for (int i = 0; i < route.waypoints.Length; i++)
            {
                var point = route.waypoints[i];
                Vector3 position = line.GetPosition(i * 100);
                Check(position.x == point.x && position.y == point.y, "Waypoint coordinate mismatch");
                var marker = visuals.Find("Waypoint_" + point.id).GetComponent<LineRenderer>();
                Color expected = i == 0 ? Color.green : i == 4 ? Color.red : new Color(1f, 0.5f, 0f);
                Check(marker.loop && marker.sharedMaterial.color == expected, "Marker color mismatch");
                var label = visuals.Find("Label_" + point.id).GetComponent<TextMeshPro>();
                Check(label.text == point.id && label.font == display.fontAsset, "Label/font mismatch");
            }
            temporaryBoat = new GameObject("Route validation temporary boat");
            var boat = temporaryBoat.AddComponent<BoatDynamics2D>();
            typeof(BoatDynamics2D).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance)
                .Invoke(boat, null);
            boat.SetMode(BoatDynamics2D.ControlMode.External);
            boat.ResetBoat();
            Check(display.IsLoaded && line.positionCount == 401 && root.transform.parent == null,
                "Reset or mode change removed route");
            typeof(TargetRouteDisplay).GetMethod("OnDisable", BindingFlags.NonPublic | BindingFlags.Instance)
                .Invoke(display, null);
            Check(!display.IsLoaded && root.transform.childCount == 0, "Generated visuals not cleaned up");
            Check(display.LoadRoute() && root.transform.childCount == 1, "Reload duplicated or lost visuals");
            typeof(TargetRouteDisplay).GetMethod("OnDisable", BindingFlags.NonPublic | BindingFlags.Instance)
                .Invoke(display, null);
            UnityEngine.Object.DestroyImmediate(temporaryBoat); temporaryBoat = null;
            Debug.Log("TARGET_ROUTE_VALIDATION_PASS: scene wiring, 401 curve samples, five task points, colors, labels, layer, reset/mode independence, cleanup and reload. No scene saved.");
            EditorApplication.Exit(0);
        }
        catch (Exception exception)
        {
            Debug.LogException(exception);
            if (temporaryBoat != null) UnityEngine.Object.DestroyImmediate(temporaryBoat);
            if (display != null)
                typeof(TargetRouteDisplay).GetMethod("OnDisable", BindingFlags.NonPublic | BindingFlags.Instance)
                    .Invoke(display, null);
            EditorApplication.Exit(1);
        }
    }
}
