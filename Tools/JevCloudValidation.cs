// Independent test copy only. HTTP is intercepted by the validation launcher.
using System;
using System.Collections;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using Newtonsoft.Json.Linq;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
#endif

public sealed class JevCloudValidation : MonoBehaviour
{
#if UNITY_EDITOR
    string output;
    bool failed;
    BoatDynamics2D boat;
    LocalModelSwitcher model;
    SimulationRunner runner;
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
        if (Array.IndexOf(Environment.GetCommandLineArgs(), "--jev-cloud-validation") >= 0)
            new GameObject("JevCloudValidation").AddComponent<JevCloudValidation>();
    }
    void Check(string stage, bool passed, string detail = null)
    {
        failed |= !passed;
        File.AppendAllText(output, new JObject { ["stage"] = stage, ["passed"] = passed,
            ["detail"] = detail }.ToString(Newtonsoft.Json.Formatting.None) + "\n");
    }
    static bool PortFree(int port)
    {
        try { using (var udp = new UdpClient()) { udp.ExclusiveAddressUse = true;
            udp.Client.Bind(new IPEndPoint(IPAddress.Loopback, port)); } return true; }
        catch (SocketException) { return false; }
    }
    IEnumerator Start()
    {
        output = Environment.GetEnvironmentVariable("USV_JEV_VALIDATION_OUTPUT");
        yield return null;
        boat = UnityEngine.Object.FindAnyObjectByType<BoatDynamics2D>();
        runner = boat.GetComponent<SimulationRunner>();
        model = boat.GetComponent<LocalModelSwitcher>();
        model.projectRoot = Environment.GetEnvironmentVariable("USV_JEV_PROJECT");
        boat.GetComponent<MissionManager>().SelectMapLevel(0);
        Environment.SetEnvironmentVariable("JEV_API_KEY", null);
        string run = boat.RunId;
        model.SelectModel("cloud");
        Check("missing_key_explicit_error", !model.IsSwitching && model.Status.Contains("JEV_API_KEY"));
        Check("missing_key_keeps_current_run", run == boat.RunId);
        string localCredentials = Path.Combine(model.projectRoot, ".env.jev.json");
        var readKey = typeof(LocalModelSwitcher).GetMethod("CloudKey", BindingFlags.Instance | BindingFlags.NonPublic);
        File.WriteAllText(localCredentials, "{\"JEV_API_KEY\": 123}");
        Check("invalid_local_key_is_rejected", readKey.Invoke(model, null) == null);
        File.WriteAllText(localCredentials, "{invalid json");
        Check("malformed_local_config_is_rejected", readKey.Invoke(model, null) == null);
        File.WriteAllText(localCredentials, "{\"JEV_API_KEY\": \" validation-only-no-cloud-access \"}");
        Check("local_key_loaded_without_environment", (string)readKey.Invoke(model, null) == "validation-only-no-cloud-access");
        Environment.SetEnvironmentVariable("JEV_API_KEY", "environment-override-test");
        Check("environment_key_takes_precedence", (string)readKey.Invoke(model, null) == "environment-override-test");
        Environment.SetEnvironmentVariable("JEV_API_KEY", null);
        boat.maxAcceleration = .3f; boat.maxForwardSpeed = 7; boat.maxReverseSpeed = 2;
        boat.maxAngularAccelerationDeg = 120; boat.maxYawRateDeg = 80;
        boat.accelerationTimeConstant = .25f; boat.angularAccelerationTimeConstant = .3f;
        boat.linearDamping = .9f; boat.angularDamping = 3;
        runner.actionDuration = .8f; Time.fixedDeltaTime = .04f;
        model.SelectModel("cloud");
        Check("repository_parameters_restored", boat.maxAcceleration == 1.5f && boat.maxForwardSpeed == 5
            && boat.maxReverseSpeed == 1 && boat.maxAngularAccelerationDeg == 60 && boat.maxYawRateDeg == 45
            && boat.accelerationTimeConstant == 0 && boat.angularAccelerationTimeConstant == 0
            && boat.linearDamping == .2f && boat.angularDamping == 1.2f
            && runner.actionDuration == .4f && Mathf.Abs(Time.fixedDeltaTime - .02f) < .000001f,
            JsonUtility.ToJson(BoatParameters.Capture(boat)) + "; duration=" + runner.actionDuration);
        Check("new_run_reset", boat.Tick == 0 && run != boat.RunId);
        double deadline = Time.realtimeSinceStartupAsDouble + 60;
        while (model.IsSwitching && Time.realtimeSinceStartupAsDouble < deadline) yield return null;
        Check("cloud_client_ready", model.IsReady && model.ActiveBackend == "cloud", model.Status);
        if (!model.IsReady) { model.enabled = false; EditorApplication.Exit(2); yield break; }
        Check("actual_model_label", model.ActiveModelName == "Jev (cloud)");
        deadline = Time.realtimeSinceStartupAsDouble + 20;
        while (boat.Tick < 40 && runner.State != SimulationRunner.RunState.Failed
            && Time.realtimeSinceStartupAsDouble < deadline) yield return null;
        Check("mock_response_moves_real_unity_boat_forward", boat.Tick >= 40 && boat.Speed > 0
            && boat.Position.x > 0, runner.Status);
        boat.ResetBoat();
        yield return null;
        Check("reset_pauses_cloud_run", !runner.JevStarted && boat.Tick == 0);
        runner.StartJev();
        deadline = Time.realtimeSinceStartupAsDouble + 15;
        while (boat.Tick < 20 && runner.State != SimulationRunner.RunState.Failed
            && Time.realtimeSinceStartupAsDouble < deadline) yield return null;
        Check("resume_keeps_cloud_model", boat.Tick >= 20 && model.ActiveBackend == "cloud");
        File.WriteAllText(Path.Combine(Path.GetDirectoryName(output), "fail-http"), "fail");
        deadline = Time.realtimeSinceStartupAsDouble + 15;
        while (runner.State != SimulationRunner.RunState.Failed && Time.realtimeSinceStartupAsDouble < deadline)
            yield return null;
        long tick = boat.Tick;
        for (int i = 0; i < 10; i++) yield return null;
        Check("cloud_http_error_pauses_without_fallback", runner.State == SimulationRunner.RunState.Failed
            && boat.Tick == tick && model.ActiveBackend == "cloud", runner.Status);
        model.enabled = false;
        deadline = Time.realtimeSinceStartupAsDouble + 20;
        while (!PortFree(15006) && Time.realtimeSinceStartupAsDouble < deadline) yield return null;
        Check("cleanup_releases_worker_port", PortFree(15006));
        EditorApplication.Exit(failed ? 2 : 0);
    }
#endif
}
