using System;
using Newtonsoft.Json.Linq;
using UnityEngine;

[DefaultExecutionOrder(100)]
[RequireComponent(typeof(BoatDynamics2D), typeof(MissionManager), typeof(BoatActionExecutor))]
public sealed class SimulationRunner : MonoBehaviour
{
    public enum RunState { Initializing, Manual, Ready, Waiting, Executing, Completed, Failed, Stopped }
    public enum DecisionTimingMode { Paused, Continuous }
    public DecisionTimingMode DecisionMode { get; private set; } = DecisionTimingMode.Paused;
    public bool PauseWhileDeciding => DecisionMode == DecisionTimingMode.Paused;
    public bool CanChangeDecisionMode => false;
    public bool JevStarted { get; private set; }
    public bool StopRequested { get; private set; }
    [Min(0.02f)] public float actionDuration = BoatParameters.DefaultActionDuration;
    public RunState State { get; private set; } = RunState.Initializing;
    public string LastError { get; private set; }
    public string DecisionId { get; private set; }
    public event Action StepCompleted;
    public bool IsWaiting => State == RunState.Waiting || State == RunState.Failed;
    public string Status => State + (LastError == null ? "" : ": " + LastError)
        + (bridge == null || bridge.AlgorithmStatus == 0 ? "" : " | Algorithm " + bridge.AlgorithmStatus + ": " + bridge.AlgorithmDescription);
    private BoatDynamics2D boat;
    private MissionManager mission;
    private BoatActionExecutor player;
    private BoatUdpBridge bridge;
    private DynamicObstacleField obstacles;
    private string identity, parameters;
    private long requestTick;
    private float requestedDuration;
    private string completedEpisodeId;
    void Awake()
    {
        boat = GetComponent<BoatDynamics2D>(); mission = GetComponent<MissionManager>();
        player = GetComponent<BoatActionExecutor>(); bridge = GetComponent<BoatUdpBridge>();
        obstacles = GetComponent<DynamicObstacleField>() ?? gameObject.AddComponent<DynamicObstacleField>();
        if (GetComponent<BoatHullVisual>() == null) gameObject.AddComponent<BoatHullVisual>();
        if (GetComponent<BoatTrajectoryRecorder>() == null) gameObject.AddComponent<BoatTrajectoryRecorder>();
    }
    void OnEnable()
    {
        boat.ResetPerformed += StopJev; boat.ModeChanged += StopJev;
        if (bridge != null) bridge.DecisionReceived += ReceiveDecision;
    }
    void OnDisable()
    {
        boat.ResetPerformed -= StopJev; boat.ModeChanged -= StopJev;
        if (bridge != null) bridge.DecisionReceived -= ReceiveDecision;
        StopJev();
    }
    void Start()
    {
        // Optional desktop startup uses the same path as the Start Jev button.
        var args = Environment.GetCommandLineArgs();
        if (Array.IndexOf(args, "--jev") >= 0) { boat.SetMode(BoatDynamics2D.ControlMode.Jev); StartJev(); }
    }
    public void StartJev()
    {
        if (mission.Failed || boat.Mode != BoatDynamics2D.ControlMode.Jev || !mission.HasTask || mission.Completed) return;
        Invalidate(); StopRequested = false; JevStarted = true; Pump();
    }
    public void RequestImmediateStop()
    {
        if (mission.Failed || boat.Mode != BoatDynamics2D.ControlMode.Jev || !JevStarted || StopRequested
            || !mission.HasTask || mission.Completed) return;
        Invalidate(); StopRequested = true; BeginDecision();
    }
    public void StopJev() { JevStarted = false; StopRequested = false; Invalidate(); }
    public void Retry() { Invalidate(); }
    // 预留统一设置入口；不暂停调度尚未实现，不能只切换标签或协议标志。
    public bool TrySetDecisionMode(DecisionTimingMode mode)
    {
        if (mode != DecisionTimingMode.Paused || JevStarted) return false;
        DecisionMode = mode;
        return true;
    }
    public void Invalidate()
    {
        player.Cancel(); bridge?.CancelDecisionRequest();
        DecisionId = null; identity = null; LastError = null; State = RunState.Initializing;
        if (mission.Failed) { LastError = mission.FailureReason; State = RunState.Failed; }
    }
    private string Identity() => boat.SessionId + "/" + boat.EpisodeId + "/" + boat.RunId + "/" + mission.Revision;
    private string Parameters() => JsonUtility.ToJson(BoatParameters.Capture(boat)) + "/" + (obstacles == null ? "" : JsonUtility.ToJson(obstacles.Settings)) + "/" + actionDuration.ToString("R", System.Globalization.CultureInfo.InvariantCulture);
    void Update() { Pump(); }
    void FixedUpdate() { Advance(Time.fixedDeltaTime); }
    public void Pump()
    {
        mission.SynchronizeHull();
        bridge?.PumpIncoming();
        if (mission.Failed) { State = RunState.Failed; LastError = mission.FailureReason; return; }
        if (!mission.HasTask) { State = RunState.Initializing; return; }
        if (mission.Completed && !StopRequested && State != RunState.Stopped)
        { CompleteMission(); return; }
        if (boat.Mode != BoatDynamics2D.ControlMode.Jev)
        { State = RunState.Manual; return; }
        if (!JevStarted) { if (State != RunState.Stopped) State = RunState.Ready; return; }
        if (State == RunState.Failed) return;
        if (identity != null && (identity != Identity() || parameters != Parameters())) Invalidate();
        if (!player.HasAction && State != RunState.Waiting) BeginDecision();
    }
    private void BeginDecision()
    {
        try
        {
            if (bridge == null || !bridge.Listening) throw new InvalidOperationException("UDP bridge unavailable.");
            DiscreteAction.Steps(actionDuration, Time.fixedDeltaTime);
            requestedDuration = actionDuration; requestTick = boat.Tick;
            identity = Identity(); parameters = Parameters(); DecisionId = Guid.NewGuid().ToString("N");
            var observation = StopRequested
                ? ObservationBuilder.ForStop(boat, mission, requestedDuration)
                : ObservationBuilder.ForModel(boat, mission, requestedDuration);
            var packet = Envelope(StopRequested ? "stop_request" : "decision_request"); packet["observation"] = observation;
            bridge.QueuePacket(packet);
            State = RunState.Waiting;
        }
        catch (Exception error) { Fail(error.Message); }
    }
    private JObject Envelope(string type) => new JObject {
        ["type"] = type, ["session_id"] = boat.SessionId, ["episode_id"] = boat.EpisodeId,
        ["run_id"] = boat.RunId, ["decision_id"] = DecisionId, ["tick"] = requestTick };
    public void ReceiveDecision(JObject packet)
    {
        if (mission.Failed || boat.Mode != BoatDynamics2D.ControlMode.Jev || DecisionId == null || identity != Identity()
            || (string)packet["episode_id"] != boat.EpisodeId || (string)packet["run_id"] != boat.RunId
            || (string)packet["decision_id"] != DecisionId || (long?)packet["tick"] != requestTick) return;
        if (State != RunState.Waiting) return;
        try
        {
            if (parameters != Parameters()) { Invalidate(); return; }
            if ((string)packet["type"] == "decision_error") throw new InvalidOperationException((string)packet["error"] ?? "Python decision failed.");
            if ((string)packet["type"] != "decision_result") return;
            if (boat.Tick != requestTick) throw new InvalidOperationException("Simulation moved while waiting.");
            if (packet["actions"] != null || packet["action"] is not JObject action || action.Count != 2)
                throw new FormatException("Expected one action object.");
            foreach (string field in new[] { "speed", "course" })
                if (action[field]?.Type != JTokenType.Float && action[field]?.Type != JTokenType.Integer)
                    throw new FormatException("Missing numeric action field: " + field);
            // The matching request owns its duration; the response carries only the command.
            player.AcceptAction(action.ToObject<DiscreteBoatAction>(), requestedDuration, Time.fixedDeltaTime);
            bridge.CancelDecisionRequest(); State = RunState.Executing;
        }
        catch (Exception error) { Fail(error.Message); }
    }
    private void Fail(string error)
    { player.Cancel(); bridge?.CancelDecisionRequest(); LastError = error; State = RunState.Failed; }
    private void CompleteMission()
    {
        if (State == RunState.Completed) return;
        player.Cancel(); bridge?.CancelDecisionRequest(); State = RunState.Completed;
        // 同一任务轮次只通知一次；切换控制模式不会重复报告已经完成的任务。
        if (completedEpisodeId == boat.EpisodeId) return;
        completedEpisodeId = boat.EpisodeId;
        NotifySimulation("mission_completed");
    }
    private void NotifySimulation(string type)
    {
        bridge?.QueuePacket(new JObject {
            ["type"] = type, ["session_id"] = boat.SessionId, ["episode_id"] = boat.EpisodeId,
            ["run_id"] = boat.RunId, ["tick"] = boat.Tick,
            ["params"] = mission.IsAccompany ? mission.Accompany.StatusParams() : mission.IsFollowing ? new JObject { ["platCode"] = 1001, ["targetPlatformId"] = FollowingTarget.PlatformId }
                : new JObject { ["platCode"] = 1001, ["currentRoute"] = Math.Max(0, (mission.Progress?.NextIndex ?? 1) - 1) }
        });
    }
    public bool Advance(float dt)
    {
        Pump();
        if (mission.Failed || !mission.HasTask || (mission.Completed && !StopRequested) || State == RunState.Failed) return false;
        if (boat.Mode == BoatDynamics2D.ControlMode.Jev)
        {
            try { if (!player.PrepareStep(dt)) return false; }
            catch (Exception error) { Fail(error.Message); return false; }
        }
        Vector2 previous = boat.Position; float previousSpeed = boat.Speed;
        boat.Step(dt);
        string collision = obstacles.Step(dt, previous, boat.Position);
        if (mission.IsFollowing)
        {
            var previousTarget = mission.Following.Position;
            collision = mission.Following.Step(dt, previous, boat.Position) ?? collision;
            collision = obstacles.MeasureFollowingTarget(previousTarget, mission.Following.Position) ?? collision;
        }
        if (mission.IsAccompany)
            collision = mission.Accompany.Step(dt, previous, boat.Position, StopRequested) ?? collision;
        if (collision != null)
        {
            mission.FailCollision(collision);
            NotifySimulation("simulation_collision"); Invalidate();
            StepCompleted?.Invoke(); return true;
        }
        if (!StopRequested) mission.Evaluate(previous, previousSpeed);
        if (boat.Mode == BoatDynamics2D.ControlMode.Jev)
        { player.FinishStep(); if (!player.HasAction) State = RunState.Initializing; }
        if (StopRequested && Mathf.Abs(boat.Speed) <= ObservationBuilder.ImmediateStopSpeedMps)
        {
            player.Cancel(); bridge?.CancelDecisionRequest();
            DecisionId = null; identity = null; StopRequested = false;
            JevStarted = false; State = RunState.Stopped;
            NotifySimulation("stop_completed");
        }
        if (mission.Completed && !StopRequested && State != RunState.Stopped) CompleteMission();
        StepCompleted?.Invoke();
        return true;
    }
}
