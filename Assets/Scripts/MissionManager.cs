using System;
using System.IO;
using UnityEngine;

[RequireComponent(typeof(BoatDynamics2D))]
public sealed class MissionManager : MonoBehaviour
{
    public const int MaxMapLevel = 4;
    public enum TaskMode { Route, Following }
    public TaskMode Mode { get; private set; }
    public bool IsFollowing => Mode == TaskMode.Following;
    public FollowingTarget Following { get; private set; }
    public bool HasTask => IsFollowing ? Following?.Scenario != null : Progress != null;
    public bool Completed => IsFollowing ? Following.Completed : Progress != null && Progress.Completed;
    public string TaskCode => IsFollowing ? "targetFollowing" : "trackingMode";
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

    void Awake() { boat = GetComponent<BoatDynamics2D>(); Following = GetComponent<FollowingTarget>() ?? gameObject.AddComponent<FollowingTarget>(); }
    void OnEnable() { boat.ResetPerformed += ResetProgress; }
    void OnDisable() { boat.ResetPerformed -= ResetProgress; }
    void Start() { if (!HasTask) Reload(); }

    public bool Reload()
    {
        if (IsFollowing) return SelectFollowingScenario(Following.ScenarioIndex);
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
            Mode = TaskMode.Route; Following.ClearVisuals(); Route = next;
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

    public bool SelectTaskMode(TaskMode mode) => mode == Mode || (mode == TaskMode.Following
        ? SelectFollowingScenario(0) : SelectMapLevel(MapLevel));
    public bool SelectFollowingScenario(int index)
    {
        try
        {
            if (index < 0 || index >= FollowingTarget.ScenarioFiles.Length) throw new FormatException("Invalid following scenario.");
            var next = FollowingScenarioData.Parse(File.ReadAllText(Path.Combine(Application.streamingAssetsPath,
                "Following", FollowingTarget.ScenarioFiles[index] + ".json")));
            if (next.initial_state.speed < -boat.maxReverseSpeed || next.initial_state.speed > boat.maxForwardSpeed
                || Mathf.Abs(next.initial_state.yaw_rate_deg) > boat.maxYawRateDeg)
                throw new FormatException("Initial velocity exceeds boat limits.");
            Following.Load(next, index);
            Mode = TaskMode.Following; Route = null; Progress = null; routeFile = "";
            Revision++; LoadError = null;
            boat.ConfigureInitialState(next.initial_state); boat.ResetBoat(); RouteChanged?.Invoke(); return true;
        }
        catch (Exception error) { LoadError = error.Message; return false; }
    }

    public void SynchronizeHull() { if (Route != null) Route.arrival_radius_m = boat.WaypointRadius; }
    private void ResetProgress() { SynchronizeHull(); Failed = false; FailureReason = null; if (IsFollowing) Following.ResetTarget(); else Progress?.Reset(boat.Position, boat.Speed); }
    public void FailCollision(string obstacleId) { Failed = true; FailureReason = "Collision with " + obstacleId + ". Reset required."; }
    public void Evaluate(Vector2 previous, float previousSpeed) { Progress?.Evaluate(previous, boat.Position, boat.Speed, previousSpeed); }
    public string Status => Failed ? "Failed" : !HasTask ? "NoTask" : Completed ? "Completed" : IsFollowing ? Following.TrackingStatus : "Running";
}
