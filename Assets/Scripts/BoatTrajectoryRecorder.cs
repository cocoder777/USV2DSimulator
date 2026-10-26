using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;

// Observe completed physical steps; never controls the boat or changes the mission judge.
[DisallowMultipleComponent]
public sealed class BoatTrajectoryRecorder : MonoBehaviour
{
    [Tooltip("Python executable with matplotlib installed. A full executable path is supported.")]
    public string pythonExecutable = "python";
    public string LastDataPath { get; private set; }
    public string LastImagePath { get; private set; }
    public string ExportError { get; private set; }
    public bool IsPlotting => jobs.Count > 0;
    public int SampleCount => samples.Count;

    [Serializable]
    private struct Sample
    {
        public long tick;
        public double simulation_time;
        public float x, y, speed, heading_deg, yaw_rate_deg_s, acceleration_mps2, angular_acceleration_deg_s2;
        public string mode;
        public JArray obstacles { get; set; }
    }

    private sealed class PlotJob
    {
        public int serial;
        public string image;
        public Task<string> result;
    }

    private readonly List<Sample> samples = new List<Sample>();
    private readonly List<PlotJob> jobs = new List<PlotJob>();
    private BoatDynamics2D boat;
    private MissionManager mission;
    private SimulationRunner runner;
    private TargetRouteData recordedRoute;
    private string episode;
    private bool exported;
    private int exportSerial;

    void Awake()
    {
        boat = GetComponent<BoatDynamics2D>();
        mission = GetComponent<MissionManager>();
        runner = GetComponent<SimulationRunner>();
    }

    void OnEnable()
    {
        boat.ResetPerformed += BeginEpisode;
        runner.StepCompleted += RecordStep;
        BeginEpisode();
    }

    void OnDisable()
    {
        boat.ResetPerformed -= BeginEpisode;
        runner.StepCompleted -= RecordStep;
    }

    private void BeginEpisode()
    {
        samples.Clear();
        exported = false;
        episode = boat.EpisodeId;
        recordedRoute = mission.Route;
        if (recordedRoute != null) AddSample();
    }

    private void AddSample()
    {
        samples.Add(new Sample { tick = boat.Tick, simulation_time = boat.SimulationTime,
            x = boat.Position.x, y = boat.Position.y, speed = boat.Speed, mode = boat.Mode.ToString(),
            heading_deg = boat.HeadingDeg, yaw_rate_deg_s = boat.YawRateDeg,
            acceleration_mps2 = boat.ActualAccelerationValue, angular_acceleration_deg_s2 = boat.ActualAngularAccelerationDeg,
            obstacles = boat.GetComponent<DynamicObstacleField>()?.Snapshot() });
    }

    private void RecordStep()
    {
        if (recordedRoute != mission.Route || episode != boat.EpisodeId) BeginEpisode();
        if (recordedRoute == null || exported) return;
        AddSample();
        if (mission.Failed || mission.Progress.Completed) ExportCompletedRun();
    }

    private void ExportCompletedRun()
    {
        exported = true; // Exactly once per episode, including a failed disk write.
        ExportError = null;
        LastImagePath = null;
        LastDataPath = null;
        try
        {
            string directory = Application.isEditor
                ? Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Logs", "trajectories"))
                : Path.Combine(Application.persistentDataPath, "Trajectories");
            Directory.CreateDirectory(directory);
            string name = "trajectory_" + DateTime.UtcNow.ToString("yyyyMMdd_HHmmss_fff") + "Z_" + episode;
            string data = Path.Combine(directory, name + ".json");
            var document = new { schema_version = 1, result = mission.Failed ? "failed" : "completed",
                failure_reason = mission.FailureReason, boat_radius_m = boat.CollisionRadius, episode_id = episode,
                map_level = RecordedMapLevel(), parameters = BoatParameters.Capture(boat), action_duration_s = runner.actionDuration,
                minimum_physical_clearance_m = GetComponent<DynamicObstacleField>()?.MinimumPhysicalClearance,
                minimum_avoidance_clearance_m = GetComponent<DynamicObstacleField>()?.MinimumAvoidanceClearance,
                route = recordedRoute, samples, simulation_time = boat.SimulationTime,
                final_speed_mps = boat.Speed, passed_count = mission.Progress.NextIndex,
                passing_max_speeds_mps = mission.Progress.PassingMaxSpeeds };
            File.WriteAllText(data, JsonConvert.SerializeObject(document, Formatting.Indented));
            LastDataPath = data;
            UnityEngine.Debug.Log("Trajectory data saved: " + data, this);
            StartPlot(data);
        }
        catch (Exception error)
        {
            ExportError = "Could not save trajectory: " + error.Message;
            UnityEngine.Debug.LogWarning(ExportError, this);
        }
    }

    private int? RecordedMapLevel()
    {
        if (recordedRoute.route_id == "usv_fixed_curve_001") return 0;
        for (int level = 1; level <= MissionManager.MaxMapLevel; level++)
            if (recordedRoute.route_id == "usv_map_level_" + level) return level;
        return null;
    }

    public void RetryPlot()
    {
        if (LastDataPath == null || IsPlotting) return;
        try { StartPlot(LastDataPath); }
        catch (Exception error)
        {
            ExportError = "Could not start plot: " + error.Message;
            UnityEngine.Debug.LogWarning(ExportError, this);
        }
    }

    private void StartPlot(string data)
    {
        ExportError = null;
        string image = Path.ChangeExtension(data, ".png");
        string script = Path.Combine(Application.streamingAssetsPath, "Tools", "plot_trajectory.py");
        string executable = pythonExecutable;
        // Unity Hub's PATH can differ from the Python terminal's PATH (especially with Conda).
        // Read configuration per export so fixing it and pressing Retry requires no Editor restart.
        string configuredPython = Environment.GetEnvironmentVariable("USV_PLOT_PYTHON");
        string configuration = Application.isEditor
            ? Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Logs", "trajectory_python.txt"))
            : Path.Combine(Application.persistentDataPath, "trajectory_python.txt");
        if (!string.IsNullOrWhiteSpace(configuredPython)) executable = configuredPython;
        else if (File.Exists(configuration)) executable = File.ReadAllText(configuration).Trim();
        jobs.Add(new PlotJob { serial = ++exportSerial, image = image,
            result = Task.Run(() => RenderPlot(executable, script, data, image)) });
    }

    // Quote one Windows command-line argument, including embedded quotes and trailing backslashes.
    private static string Quote(string value)
    {
        var quoted = new System.Text.StringBuilder("\"");
        int slashes = 0;
        foreach (char character in value)
        {
            if (character == '\\') { slashes++; continue; }
            quoted.Append('\\', character == '"' ? slashes * 2 + 1 : slashes);
            quoted.Append(character);
            slashes = 0;
        }
        quoted.Append('\\', slashes * 2);
        return quoted.Append('"').ToString();
    }

    private static string RenderPlot(string executable, string script, string data, string image)
    {
        try
        {
            using (var process = new Process { StartInfo = new ProcessStartInfo {
                FileName = executable, Arguments = Quote(script) + " " + Quote(data) + " --output " + Quote(image),
                UseShellExecute = false, CreateNoWindow = true, RedirectStandardError = true,
                WindowStyle = ProcessWindowStyle.Hidden } })
            {
                process.Start();
                string error = process.StandardError.ReadToEnd();
                process.WaitForExit();
                return process.ExitCode == 0 && File.Exists(image) ? null
                    : "Plot generation failed. JSON saved. " + error.Trim();
            }
        }
        catch (Exception error) { return "Plot generation failed. JSON saved. " + error.Message; }
    }

    void Update()
    {
        for (int i = jobs.Count - 1; i >= 0; i--)
        {
            var job = jobs[i];
            if (!job.result.IsCompleted) continue;
            string error = job.result.Result;
            jobs.RemoveAt(i);
            if (job.serial != exportSerial) continue;
            ExportError = error;
            if (error == null)
            {
                LastImagePath = job.image;
                UnityEngine.Debug.Log("Trajectory plot saved: " + LastImagePath, this);
            }
            else UnityEngine.Debug.LogWarning(error, this);
        }
    }

    public void OpenPlot()
    {
        if (!string.IsNullOrEmpty(LastImagePath) && File.Exists(LastImagePath))
            Application.OpenURL(new Uri(LastImagePath).AbsoluteUri);
    }
}
