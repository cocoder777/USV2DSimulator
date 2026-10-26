using System;
using System.IO;
using UnityEngine;

[RequireComponent(typeof(BoatDynamics2D))]
public sealed class MissionManager : MonoBehaviour
{
    public const int MaxMapLevel = 4;
    [Tooltip("Empty uses the bundled StreamingAssets task. Absolute or project-relative paths are supported on desktop.")]
    public string routeFile = "";
    public int MapLevel { get; private set; }
    public TargetRouteData Route { get; private set; }
    public MissionProgress Progress { get; private set; }
    public int Revision { get; private set; }
    public string LoadError { get; private set; }
    public bool Failed { get; private set; }
    public string FailureReason { get; private set; }
    public event Action RouteChanged;
    private BoatDynamics2D boat;

    void Awake() { boat = GetComponent<BoatDynamics2D>(); }
    void OnEnable() { boat.ResetPerformed += ResetProgress; }
    void OnDisable() { boat.ResetPerformed -= ResetProgress; }
    void Start() { if (Route == null) Reload(); }

    public bool Reload()
    {
        string path = string.IsNullOrWhiteSpace(routeFile)
            ? BundledMapPath(MapLevel)
            : Path.GetFullPath(Path.IsPathRooted(routeFile) ? routeFile
                : Path.Combine(Application.dataPath, "..", routeFile));
        try { return LoadJson(File.ReadAllText(path)); }
        catch (Exception error) { LoadError = "Task load failed: " + error.Message; Debug.LogError(LoadError, this); return false; }
    }

    private static string BundledMapPath(int level) => level == 0
        ? Path.Combine(Application.streamingAssetsPath, "fixed_route.json")
        : Path.Combine(Application.streamingAssetsPath, "Maps", "level_" + level + ".json");

    // Commit the selection only after loading/validation succeeds. A failed selection keeps the current task.
    public bool SelectMapLevel(int level)
    {
        if (level < 0 || level > MaxMapLevel) { LoadError = "Map level must be 0.." + MaxMapLevel + "."; return false; }
        try
        {
            if (!LoadJson(File.ReadAllText(BundledMapPath(level)))) return false;
            MapLevel = level;
            routeFile = "";
            return true;
        }
        catch (Exception error) { LoadError = "Map load failed: " + error.Message; return false; }
    }

    public bool LoadJson(string json)
    {
        try
        {
            var next = TargetRouteData.Parse(json);
            var initial = next.initial_state;
            if (initial.speed > boat.maxForwardSpeed || initial.speed < -boat.maxReverseSpeed
                || Mathf.Abs(initial.yaw_rate_deg) > boat.maxYawRateDeg)
                throw new FormatException("Initial velocity exceeds the current boat limits.");
            // Parse/validate before committing; an invalid replacement leaves the active task intact.
            next.arrival_radius_m = boat.WaypointRadius;
            Route = next;
            Progress = new MissionProgress(next);
            Revision++;
            LoadError = null;
            boat.ConfigureInitialState(initial);
            boat.ResetBoat();
            RouteChanged?.Invoke();
            return true;
        }
        catch (Exception error) { LoadError = error.Message; return false; }
    }

    public void SynchronizeHull() { if (Route != null) Route.arrival_radius_m = boat.WaypointRadius; }
    private void ResetProgress() { SynchronizeHull(); Failed = false; FailureReason = null; Progress?.Reset(boat.Position, boat.Speed); }
    public void FailCollision(string obstacleId) { Failed = true; FailureReason = "Collision with " + obstacleId + ". Reset required."; }
    public void Evaluate(Vector2 previous, float previousSpeed) { Progress?.Evaluate(previous, boat.Position, boat.Speed, previousSpeed); }
    public string Status => Progress == null ? "NoTask" : Failed ? "Failed" : Progress.Completed ? "Completed" : "Running";
}
