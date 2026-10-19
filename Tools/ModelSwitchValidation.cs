// Installs only in an independent test copy. Calls the exact GUI callbacks.
using System;
using System.Collections;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using Newtonsoft.Json.Linq;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
#endif

public sealed class ModelSwitchValidation : MonoBehaviour
{
#if UNITY_EDITOR
    BoatDynamics2D boat;
    SimulationRunner runner;
    MissionManager mission;
    LocalModelSwitcher model;
    string output, parameters;
    bool failed;
    public static void Run()
    {
        AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
        EditorSettings.enterPlayModeOptionsEnabled = false;
        EditorSceneManager.OpenScene("Assets/Scenes/Main.unity");
        EditorApplication.isPlaying = true;
    }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Install()
    {
        if (Array.IndexOf(Environment.GetCommandLineArgs(), "--model-switch-validation") >= 0)
            new GameObject("ModelSwitchValidation").AddComponent<ModelSwitchValidation>();
    }
    void Record(string stage, bool passed, string detail = null)
    {
        File.AppendAllText(output, new JObject { ["stage"] = stage, ["passed"] = passed, ["detail"] = detail,
            ["tick"] = boat == null ? 0 : boat.Tick, ["backend"] = model?.ActiveBackend,
            ["status"] = model?.Status }.ToString(Newtonsoft.Json.Formatting.None) + "\n");
        if (!passed) failed = true;
    }
    IEnumerator Ready(string backend)
    {
        double deadline = Time.realtimeSinceStartupAsDouble + 180;
        while (model.IsSwitching && Time.realtimeSinceStartupAsDouble < deadline) yield return null;
        Record("ready_" + backend, model.IsReady && model.ActiveBackend == backend, model.Status);
        if (failed) yield break;
        deadline = Time.realtimeSinceStartupAsDouble + 30;
        while (boat.Tick < 40 && runner.State != SimulationRunner.RunState.Failed && Time.realtimeSinceStartupAsDouble < deadline) yield return null;
        Record("motion_" + backend, boat.Tick >= 40 && model.LastDecisionSeconds >= 0, runner.Status);
        Record("parameters_" + backend, JsonUtility.ToJson(BoatParameters.Capture(boat)) == parameters && Math.Abs(runner.actionDuration - .4f) < .0001);
    }
    static bool PortFree(int port)
    {
        try { using (var udp = new UdpClient()) { udp.ExclusiveAddressUse = true; udp.Client.Bind(new IPEndPoint(IPAddress.Loopback, port)); } return true; }
        catch (SocketException) { return false; }
    }
    IEnumerator Start()
    {
        output = Environment.GetEnvironmentVariable("USV_SWITCH_OUTPUT");
        yield return null;
        boat = UnityEngine.Object.FindAnyObjectByType<BoatDynamics2D>();
        runner = boat.GetComponent<SimulationRunner>();
        mission = boat.GetComponent<MissionManager>();
        model = boat.GetComponent<LocalModelSwitcher>();
        model.projectRoot = Environment.GetEnvironmentVariable("USV_SWITCH_PROJECT");
        mission.SelectMapLevel(0);
        parameters = JsonUtility.ToJson(BoatParameters.Capture(boat));
        runner.actionDuration = .4f;
        model.SelectModel("laya");
        yield return Ready("laya");
        if (failed) { EditorApplication.Exit(2); yield break; }
        var stale = new JObject { ["type"] = "decision_result", ["version"] = 5, ["session_id"] = boat.SessionId,
            ["episode_id"] = boat.EpisodeId, ["run_id"] = boat.RunId, ["decision_id"] = runner.DecisionId,
            ["tick"] = boat.Tick, ["action_duration_s"] = .4,
            ["action"] = new JObject { ["acceleration_index"] = 7, ["turn_index"] = 6 } };
        string oldEpisode = boat.EpisodeId;
        model.SelectModel("kev");
        runner.ReceiveDecision(stale);
        Record("switch_resets_and_rejects_old_action", boat.Tick == 0 && boat.EpisodeId != oldEpisode && !boat.GetComponent<BoatActionExecutor>().HasAction);
        string previousData = boat.GetComponent<BoatTrajectoryRecorder>().LastDataPath;
        Record("partial_export_tagged_laya", previousData != null && (string)JObject.Parse(File.ReadAllText(previousData))["controller"] == "laya");
        yield return Ready("kev");
        if (failed) { EditorApplication.Exit(2); yield break; }
        model.SelectModel("laya");
        yield return Ready("laya");
        if (failed) { EditorApplication.Exit(2); yield break; }
        string log = model.WorkerLogPath;
        oldEpisode = boat.EpisodeId;
        model.RestartRun();
        Record("restart_reuses_client", model.WorkerLogPath == log && boat.EpisodeId != oldEpisode && boat.Tick == 0);
        yield return Ready("laya");
        boat.ResetBoat();
        for (int i = 0; i < 10; i++) yield return null;
        Record("R_reset_does_not_autostart", boat.Tick == 0 && !runner.JevStarted);
        boat.SetMode(BoatDynamics2D.ControlMode.Manual);
        Record("manual_mode_retained", !runner.JevStarted);
        boat.SetMode(BoatDynamics2D.ControlMode.Jev);
        boat.ResetBoat();
        runner.StartJev();
        yield return Ready("laya");
        if (failed) { EditorApplication.Exit(2); yield break; }
        using (var stop = Process.Start(new ProcessStartInfo { FileName = "powershell.exe",
            Arguments = "-NoProfile -ExecutionPolicy Bypass -File D:/AI/Laya/stop-laya.ps1", UseShellExecute = false,
            CreateNoWindow = true, WindowStyle = ProcessWindowStyle.Hidden }))
        { while (!stop.HasExited) yield return null; }
        double failureDeadline = Time.realtimeSinceStartupAsDouble + 15;
        while (runner.State != SimulationRunner.RunState.Failed && Time.realtimeSinceStartupAsDouble < failureDeadline) yield return null;
        long pausedTick = boat.Tick;
        for (int i = 0; i < 10; i++) yield return null;
        Record("service_failure_pauses_no_fallback", runner.State == SimulationRunner.RunState.Failed && boat.Tick == pausedTick, runner.Status);
        model.SelectModel("laya");
        yield return Ready("laya");
        if (failed) { EditorApplication.Exit(2); yield break; }
        log = model.WorkerLogPath;
        model.enabled = false; // Equivalent Play-mode cleanup path.
        double deadline = Time.realtimeSinceStartupAsDouble + 40;
        while (!PortFree(15006) && Time.realtimeSinceStartupAsDouble < deadline) yield return null;
        Record("disable_releases_worker_udp", PortFree(15006));
        string statusFile = Path.Combine(Path.GetDirectoryName(log), "status.json");
        while ((string)JObject.Parse(File.ReadAllText(statusFile))["phase"] != "stopped" && Time.realtimeSinceStartupAsDouble < deadline) yield return null;
        Record("owned_session_cleanup", (string)JObject.Parse(File.ReadAllText(statusFile))["phase"] == "stopped");
        EditorApplication.Exit(failed ? 2 : 0);
    }
#endif
}
