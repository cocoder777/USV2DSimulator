using System;
using System.Collections;
using System.Diagnostics;
using System.IO;
using Newtonsoft.Json.Linq;
using UnityEngine;

// Button-owned clients for local models and cloud Jev. Motion remains UDP v5.
[DisallowMultipleComponent]
public sealed class LocalModelSwitcher : MonoBehaviour
{
    public string deploymentRoot = "D:/AI";
    [Tooltip("Empty uses the Editor project root. Builds require a checkout with Tools and jev_api.")]
    public string projectRoot = "";
    public bool IsSwitching { get; private set; }
    public bool IsManaged => helper != null;
    public bool IsReady { get; private set; }
    public string ActiveBackend { get; private set; }
    public string ActiveModelName => ActiveBackend == "cloud" ? "Jev (cloud)" : ActiveBackend;
    public string Status { get; private set; } = "Select Jev, Laya or Kev to reset and run.";
    public string WorkerLogPath { get; private set; }
    public float LastDecisionSeconds { get; private set; } = -1;
    public string LastAction { get; private set; } = "--";
    private Process helper;
    private string sessionDirectory, requestedBackend, pendingRun, recordedBackend;
    private int pendingRevision;
    private bool readyHandled, destroying, pollSession;
    private double nextPoll;
    private BoatDynamics2D boat;
    private SimulationRunner runner;
    private MissionManager mission;

    void Awake()
    {
        boat = GetComponent<BoatDynamics2D>();
        runner = GetComponent<SimulationRunner>();
        mission = GetComponent<MissionManager>();
    }

    void OnEnable() { destroying = false; }

    public void SelectModel(string backend)
    {
        if (IsSwitching || destroying || (backend != "laya" && backend != "kev" && backend != "cloud")) return;
        if (Application.platform != RuntimePlatform.WindowsEditor && Application.platform != RuntimePlatform.WindowsPlayer)
        { Status = "Model buttons currently require Windows and a configured Python environment."; return; }
        if (mission?.Route == null || runner == null) { Status = "Load a valid task first."; return; }
        if (backend == "cloud" && string.IsNullOrWhiteSpace(CloudKey()))
        { Status = "Cloud Jev requires JEV_API_KEY in the environment or project .env.jev.json."; return; }
        SavePreviousRun("model_switched");
        runner.StopJev();
        if (backend == "cloud") ApplyRepositoryJevConfiguration();
        boat.SetMode(BoatDynamics2D.ControlMode.Jev);
        boat.ResetBoat();
        pendingRun = boat.RunId;
        pendingRevision = mission.Revision;
        IsReady = false;
        ActiveBackend = null;
        recordedBackend = null;
        LastDecisionSeconds = -1;
        LastAction = "--";
        IsSwitching = true;
        pollSession = false;
        requestedBackend = backend;
        Status = "Stopping the previous client...";
        StartCoroutine(SwitchModel());
    }

    IEnumerator SwitchModel()
    {
        SignalStop();
        double deadline = Time.realtimeSinceStartupAsDouble + 40;
        while (helper != null && !HasExited(helper))
        {
            if (Time.realtimeSinceStartupAsDouble >= deadline)
            { Failure("Previous client is still closing. Wait and click the model again."); yield break; }
            yield return null;
        }
        helper?.Dispose();
        helper = null;
        try
        {
            string root = Path.GetFullPath(string.IsNullOrWhiteSpace(projectRoot)
                ? Path.Combine(Application.dataPath, "..") : projectRoot);
            string modelDirectory = Path.Combine(deploymentRoot, requestedBackend == "kev" ? "Kev-0.8B" : "Laya");
            string python = Path.Combine(modelDirectory, ".venv", "Scripts", "python.exe");
            // Cloud needs only a Python interpreter, never local weights or a model service.
            if (requestedBackend == "cloud" && !File.Exists(python))
                python = Path.Combine(deploymentRoot, "Kev-0.8B", ".venv", "Scripts", "python.exe");
            string script = Path.Combine(root, "Tools", "local_model_session.py");
            if (!File.Exists(python) || !File.Exists(script))
                throw new FileNotFoundException("Missing deployment Python or Tools/local_model_session.py. Check project/deployment paths.");
            sessionDirectory = Path.Combine(root, "Logs", "model-switch", DateTime.UtcNow.ToString("yyyyMMdd_HHmmss_fff") + "_" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(sessionDirectory);
            var bridge = GetComponent<BoatUdpBridge>();
            if (bridge == null || bridge.targetAddress != "127.0.0.1")
                throw new InvalidOperationException("Model buttons require a UDP bridge targeting 127.0.0.1.");
            var info = new ProcessStartInfo {
                FileName = python,
                Arguments = "-u " + Quote(script) + " --backend " + requestedBackend
                    + " --ai-root " + Quote(Path.GetFullPath(deploymentRoot)) + " --project " + Quote(root)
                    + " --session-dir " + Quote(sessionDirectory)
                    + " --parent-pid " + Process.GetCurrentProcess().Id
                    + " --unity-port " + bridge.port + " --worker-port " + bridge.targetPort,
                WorkingDirectory = root, UseShellExecute = false, CreateNoWindow = true,
                WindowStyle = ProcessWindowStyle.Hidden
            };
            if (requestedBackend == "cloud") info.EnvironmentVariables["JEV_API_KEY"] = CloudKey();
            helper = Process.Start(info);
            pollSession = true;
            readyHandled = false;
            nextPoll = 0;
            Status = "Preparing " + requestedBackend + "...";
        }
        catch (Exception error) { Failure(error.Message); }
    }

    string CloudKey()
    {
        foreach (var target in new[] { EnvironmentVariableTarget.Process, EnvironmentVariableTarget.User,
            EnvironmentVariableTarget.Machine })
        {
            string key = Environment.GetEnvironmentVariable("JEV_API_KEY", target);
            if (!string.IsNullOrWhiteSpace(key)) return key.Trim();
        }
        // Keep checkout credentials outside tracked source. Re-read on each click
        // so an open Unity Editor can use a newly configured key immediately.
        try
        {
            string root = Path.GetFullPath(string.IsNullOrWhiteSpace(projectRoot)
                ? Path.Combine(Application.dataPath, "..") : projectRoot);
            string path = Path.Combine(root, ".env.jev.json");
            if (!File.Exists(path)) return null;
            JToken key = JObject.Parse(File.ReadAllText(path))["JEV_API_KEY"];
            return key?.Type == JTokenType.String ? ((string)key).Trim() : null;
        }
        catch (IOException) { return null; }
        catch (UnauthorizedAccessException) { return null; }
        catch (Newtonsoft.Json.JsonException) { return null; }
    }

    void ApplyRepositoryJevConfiguration()
    {
        // GitHub master 42088e5: Main.unity and ProjectSettings/TimeManager.asset.
        boat.maxAcceleration = 1.5f;
        boat.maxForwardSpeed = 5f;
        boat.maxReverseSpeed = 1f;
        boat.maxAngularAccelerationDeg = 60f;
        boat.maxYawRateDeg = 45f;
        boat.accelerationTimeConstant = 0f;
        boat.angularAccelerationTimeConstant = 0f;
        boat.linearDamping = .2f;
        boat.angularDamping = 1.2f;
        runner.actionDuration = .4f;
        Time.fixedDeltaTime = .02f;
    }

    static string Quote(string value)
    {
        int trailing = value.Length - value.TrimEnd('\\').Length;
        return "\"" + value.Replace("\"", "\\\"") + new string('\\', trailing) + "\"";
    }
    static bool HasExited(Process process)
    {
        try { return process.HasExited; }
        catch (InvalidOperationException) { return true; }
    }

    void Update()
    {
        if (!pollSession || helper == null || Time.realtimeSinceStartupAsDouble < nextPoll) return;
        nextPoll = Time.realtimeSinceStartupAsDouble + .2;
        string path = Path.Combine(sessionDirectory, "status.json");
        try
        {
            if (File.Exists(path))
            {
                JObject status;
                using (var reader = new StreamReader(new FileStream(path, FileMode.Open, FileAccess.Read,
                    FileShare.ReadWrite | FileShare.Delete))) status = JObject.Parse(reader.ReadToEnd());
                if ((string)status["backend"] != requestedBackend) return;
                Status = (string)status["message"];
                WorkerLogPath = (string)status["worker_log"];
                LastDecisionSeconds = (float?)status["last_decision_s"] ?? -1;
                LastAction = (string)status["last_action"] ?? "--";
                string phase = (string)status["phase"];
                if (phase == "error" || phase == "stopped") { Failure(Status); return; }
                if (phase == "ready" && !readyHandled)
                {
                    readyHandled = true;
                    IsSwitching = false;
                    IsReady = true;
                    ActiveBackend = requestedBackend;
                    recordedBackend = requestedBackend;
                    // Reset/map/mode changes during loading cancel automatic motion.
                    if (boat.RunId == pendingRun && mission.Revision == pendingRevision && boat.Mode == BoatDynamics2D.ControlMode.Jev)
                        runner.StartJev();
                }
            }
            if (HasExited(helper) && (IsSwitching || IsReady)) Failure("Model session exited. Inspect " + sessionDirectory);
        }
        catch (IOException) { /* Atomic status replacement may briefly share a file on Windows. */ }
        catch (Exception error) { Failure(error.Message); SignalStop(); }
    }

    void Failure(string message)
    {
        IsSwitching = false;
        IsReady = false;
        ActiveBackend = null;
        Status = message;
        runner?.StopJev();
    }

    public void RestartRun()
    {
        if (!IsReady || IsSwitching) return;
        SavePreviousRun("run_restarted");
        runner.StopJev();
        boat.SetMode(BoatDynamics2D.ControlMode.Jev);
        boat.ResetBoat();
        runner.StartJev();
    }

    void SavePreviousRun(string reason)
    {
        if (boat != null && boat.Tick > 0 && (recordedBackend != null || reason == "model_switched"))
            GetComponent<BoatTrajectoryRecorder>()?.ExportComparisonRun(recordedBackend ?? "external", reason);
    }

    void SignalStop()
    {
        if (sessionDirectory == null) return;
        try { File.WriteAllText(Path.Combine(sessionDirectory, "stop"), "stop"); }
        catch (IOException error) { UnityEngine.Debug.LogWarning("Could not signal model client: " + error.Message, this); }
    }

    void OnDisable()
    {
        destroying = true;
        StopAllCoroutines();
        SavePreviousRun("play_stopped");
        SignalStop();
        if (helper != null) runner?.StopJev();
        helper?.Dispose();
        helper = null;
        IsSwitching = false;
        IsReady = false;
        ActiveBackend = null;
        pollSession = false;
        recordedBackend = null;
    }
}
