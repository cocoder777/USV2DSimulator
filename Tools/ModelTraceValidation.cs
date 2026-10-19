// Test-copy helper only. Keep outside Assets in the working project.
using System;
using System.Collections;
using System.IO;
using Newtonsoft.Json.Linq;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
#endif

public sealed class ModelTraceValidation : MonoBehaviour
{
#if UNITY_EDITOR
    BoatDynamics2D boat;
    SimulationRunner runner;
    MissionManager mission;
    string output;
    string observedDecision;
    bool done;

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
        if (Array.IndexOf(Environment.GetCommandLineArgs(), "--model-trace-validation") >= 0)
            new GameObject("ModelTraceValidation").AddComponent<ModelTraceValidation>();
    }

    void Record(string kind, JObject data)
    {
        data["event"] = kind;
        File.AppendAllText(output, data.ToString(Newtonsoft.Json.Formatting.None) + "\n");
    }

    JObject State() => JObject.FromObject(BoatStateSnapshot.Capture(boat));

    void ObserveRequest()
    {
        if (runner.State != SimulationRunner.RunState.Waiting || runner.DecisionId == observedDecision) return;
        observedDecision = runner.DecisionId;
        var data = State();
        data["observation"] = ObservationBuilder.ForModel(boat, mission, runner.actionDuration);
        Record("request", data);
    }

    IEnumerator Start()
    {
        output = Environment.GetEnvironmentVariable("USV_TRACE_OUTPUT");
        if (string.IsNullOrEmpty(output)) { EditorApplication.Exit(2); yield break; }
        yield return null;
        boat = UnityEngine.Object.FindAnyObjectByType<BoatDynamics2D>();
        runner = boat.GetComponent<SimulationRunner>();
        mission = boat.GetComponent<MissionManager>();
        if (!mission.SelectMapLevel(0)) { Record("error", new JObject { ["message"] = mission.LoadError }); EditorApplication.Exit(2); yield break; }
        boat.ResetBoat();
        boat.GetComponent<BoatUdpBridge>().DecisionReceived += packet => Record("received", (JObject)packet.DeepClone());
        runner.StepCompleted += () =>
        {
            Record("step", State());
            if (boat.Tick >= 240) { done = true; runner.StopJev(); }
        };
        boat.SetMode(BoatDynamics2D.ControlMode.Jev);
        Record("initial", State());
        runner.StartJev();
        ObserveRequest();
        double deadline = Time.realtimeSinceStartupAsDouble + 180;
        while (!done && Time.realtimeSinceStartupAsDouble < deadline)
        {
            ObserveRequest();
            if (runner.State == SimulationRunner.RunState.Failed)
            {
                Record("error", new JObject { ["message"] = runner.LastError });
                EditorApplication.Exit(2); yield break;
            }
            yield return null;
        }
        Record("finished", new JObject { ["capped"] = done, ["state"] = State(), ["visual_validation"] = false });
        EditorApplication.Exit(done ? 0 : 2);
    }
#endif
}
