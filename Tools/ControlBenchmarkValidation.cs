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

public sealed class ControlBenchmarkValidation : MonoBehaviour
{
#if UNITY_EDITOR
    BoatDynamics2D boat;
    SimulationRunner runner;
    MissionManager mission;
    BoatUdpBridge bridge;
    string output;
    JArray scenarios;
    JObject currentScenario;
    string currentName;
    string observedDecision;
    JObject lastAction;
    bool scenarioDone;
    string endReason;
    int stepCount;
    int decisionCount;
    string lastCountedDecision;
    int scenarioCompletedCount;

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
        if (Array.IndexOf(Environment.GetCommandLineArgs(), "--control-benchmark") >= 0)
            new GameObject("ControlBenchmarkValidation").AddComponent<ControlBenchmarkValidation>();
    }

    void Record(string kind, JObject data)
    {
        data["event"] = kind;
        if (currentName != null) data["scenario"] = currentName;
        File.AppendAllText(output, data.ToString(Newtonsoft.Json.Formatting.None) + "\n");
    }

    JObject State()
    {
        var next = mission.Progress.Next;
        var data = new JObject {
            ["tick"] = boat.Tick,
            ["simulation_time"] = boat.SimulationTime,
            ["x"] = boat.Position.x,
            ["y"] = boat.Position.y,
            ["heading_deg"] = boat.HeadingDeg,
            ["speed"] = boat.Speed,
            ["yaw_rate_deg"] = boat.YawRateDeg,
            ["acceleration"] = boat.AccelerationCommandValue,
            ["angular_acceleration_deg"] = boat.AngularAccelerationCommandDeg,
            ["actual_acceleration"] = boat.ActualAccelerationValue,
            ["actual_angular_acceleration_deg"] = boat.ActualAngularAccelerationDeg,
            ["passed_count"] = mission.Progress.NextIndex,
            ["completed"] = mission.Progress.Completed,
            ["target_index"] = mission.Progress.NextIndex,
            ["target_distance"] = 0.0,
            ["bearing_deg"] = 0.0
        };
        if (next != null)
        {
            float dx = next.x - boat.Position.x, dy = next.y - boat.Position.y;
            float heading = boat.HeadingDeg * Mathf.Deg2Rad;
            float forward = dx * Mathf.Cos(heading) + dy * Mathf.Sin(heading);
            float left = -dx * Mathf.Sin(heading) + dy * Mathf.Cos(heading);
            data["target_distance"] = Mathf.Sqrt(dx * dx + dy * dy);
            data["bearing_deg"] = Mathf.Atan2(left, forward) * Mathf.Rad2Deg;
            data["target_x"] = next.x;
            data["target_y"] = next.y;
            data["target_role"] = next.role;
        }
        return data;
    }

    void ObserveRequest()
    {
        if (runner.State != SimulationRunner.RunState.Waiting || runner.DecisionId == observedDecision) return;
        observedDecision = runner.DecisionId;
        var record = State();
        record["decision_id"] = runner.DecisionId;
        record["action_duration_s"] = runner.actionDuration;
        record["observation"] = ObservationBuilder.ForModel(boat, mission, runner.actionDuration);
        Record("request", record);
    }

    void OnDecisionReceived(JObject packet)
    {
        if ((string)packet["type"] == "decision_result" && packet["action"] is JObject action)
        {
            lastAction = (JObject)action.DeepClone();
            string decisionId = (string)packet["decision_id"];
            if (decisionId != lastCountedDecision)
            {
                lastCountedDecision = decisionId;
                decisionCount++;
            }
            var record = new JObject {
                ["decision_id"] = packet["decision_id"],
                ["tick"] = packet["tick"],
                ["action"] = lastAction,
                ["action_duration_s"] = packet["action_duration_s"]
            };
            Record("decision", record);
        }
        else if ((string)packet["type"] == "decision_error")
        {
            Record("decision_error", new JObject {
                ["decision_id"] = packet["decision_id"], ["error"] = packet["error"] });
        }
    }

    void OnStep()
    {
        if (currentScenario == null) return;
        var record = State();
        if (lastAction != null)
        {
            record["acceleration_index"] = lastAction["acceleration_index"];
            record["turn_index"] = lastAction["turn_index"];
        }
        Record("step", record);
        stepCount++;
    }

    void EndScenario(string reason)
    {
        if (scenarioDone) return;
        scenarioDone = true;
        endReason = reason;
        if (reason == "completed" || reason == "checkpoint_passed") scenarioCompletedCount++;
    }

    IEnumerator RunScenario(JObject scenario)
    {
        currentScenario = scenario;
        currentName = (string)scenario["name"];
        observedDecision = null;
        lastAction = null;
        scenarioDone = false;
        endReason = null;
        stepCount = 0;
        decisionCount = 0;
        lastCountedDecision = null;
        runner.StopJev();
        var route = (JObject)scenario["route"];
        if (!mission.LoadJson(route.ToString(Newtonsoft.Json.Formatting.None)))
        {
            Record("scenario_start", new JObject { ["route"] = route, ["load_error"] = mission.LoadError });
            Record("scenario_end", new JObject {
                ["completed"] = false, ["reason"] = "load_error", ["error"] = mission.LoadError,
                ["passed_count"] = 0, ["steps"] = 0, ["simulation_time"] = 0.0 });
            currentScenario = null;
            yield break;
        }
        boat.SetMode(BoatDynamics2D.ControlMode.Jev);
        Record("scenario_start", new JObject {
            ["route"] = route,
            ["initial_state"] = route["initial_state"].DeepClone(),
            ["fixed_dt"] = Time.fixedDeltaTime,
            ["action_duration_s"] = runner.actionDuration,
            ["max_simulation_seconds"] = scenario["max_simulation_seconds"],
            ["stop_after_passed"] = scenario["stop_after_passed"]
        });
        runner.StartJev();
        double maxSimulation = (double)scenario["max_simulation_seconds"];
        int? stopAfter = (int?)scenario["stop_after_passed"];
        // Same wall allowance for every controller; simulation budgets stay unchanged.
        double wallDeadline = Time.realtimeSinceStartupAsDouble + maxSimulation * 150.0 + 300.0;
        while (!scenarioDone && Time.realtimeSinceStartupAsDouble < wallDeadline)
        {
            ObserveRequest();
            if (runner.State == SimulationRunner.RunState.Failed)
                EndScenario("runner_failed: " + runner.LastError);
            else if (mission.Progress.Completed)
                EndScenario("completed");
            else if (stopAfter.HasValue && mission.Progress.NextIndex >= stopAfter.Value)
                EndScenario("checkpoint_passed");
            else if (boat.SimulationTime >= maxSimulation)
                EndScenario("timeout");
            yield return null;
        }
        if (!scenarioDone) EndScenario("wall_timeout");
        runner.StopJev();
        var end = State();
        end["completed"] = endReason == "completed" || endReason == "checkpoint_passed";
        end["reason"] = endReason;
        end["steps"] = stepCount;
        end["decisions"] = decisionCount;
        Record("scenario_end", end);
        yield return null;
        currentScenario = null;
        currentName = null;
    }

    IEnumerator Start()
    {
        output = Environment.GetEnvironmentVariable("USV_BENCH_OUTPUT");
        string scenarioFile = Environment.GetEnvironmentVariable("USV_BENCH_SCENARIOS");
        if (string.IsNullOrEmpty(output) || string.IsNullOrEmpty(scenarioFile))
        {
            EditorApplication.Exit(2);
            yield break;
        }
        yield return null;
        boat = UnityEngine.Object.FindAnyObjectByType<BoatDynamics2D>();
        runner = boat.GetComponent<SimulationRunner>();
        mission = boat.GetComponent<MissionManager>();
        bridge = boat.GetComponent<BoatUdpBridge>();
        if (boat == null || runner == null || mission == null || bridge == null)
        {
            File.AppendAllText(output, "{\"event\":\"error\",\"message\":\"missing scene components\"}\n");
            EditorApplication.Exit(2);
            yield break;
        }
        var recorder = boat.GetComponent<BoatTrajectoryRecorder>();
        if (recorder != null) recorder.enabled = false;
        scenarios = (JArray)JObject.Parse(File.ReadAllText(scenarioFile))["scenarios"];
        bridge.DecisionReceived += OnDecisionReceived;
        runner.StepCompleted += OnStep;
        Record("benchmark_start", new JObject {
            ["controller_arg"] = "--control-benchmark",
            ["scenario_count"] = scenarios.Count,
            ["fixed_dt"] = Time.fixedDeltaTime,
            ["action_duration_s"] = runner.actionDuration
        });
        for (int index = 0; index < scenarios.Count; index++)
            yield return RunScenario((JObject)scenarios[index]);
        bridge.DecisionReceived -= OnDecisionReceived;
        runner.StepCompleted -= OnStep;
        Record("benchmark_end", new JObject {
            ["scenarios"] = scenarios.Count,
            ["completed"] = scenarioCompletedCount,
            ["unity_editor"] = true
        });
        EditorApplication.Exit(scenarioCompletedCount == scenarios.Count ? 0 : 2);
    }
#endif
}
