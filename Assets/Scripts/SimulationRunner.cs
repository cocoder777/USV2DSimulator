using System;
using Newtonsoft.Json.Linq;
using UnityEngine;

[DefaultExecutionOrder(100)]
[RequireComponent(typeof(BoatDynamics2D), typeof(MissionManager), typeof(BoatActionSequence))]
public sealed class SimulationRunner : MonoBehaviour
{
    public enum RunState { Initializing, Manual, Ready, Waiting, Executing, Completed, Failed }
    [Range(1, 10)] public int actionCount = 1;
    public float DecisionPeriod => actionCount * actionDuration;
    public bool JevStarted { get; private set; }
    [Min(0.02f)] public float actionDuration = 0.4f;
    public RunState State { get; private set; } = RunState.Initializing;
    public string LastError { get; private set; }
    public string DecisionId { get; private set; }
    public event Action StepCompleted;
    public bool IsWaiting => State == RunState.Waiting || State == RunState.Failed;
    public string Status => State + (LastError == null ? "" : ": " + LastError);
    private BoatDynamics2D boat;
    private MissionManager mission;
    private BoatActionSequence player;
    private BoatUdpBridge bridge;
    private string identity, parameters;
    private long requestTick;
    private int requestedCount;
    private float requestedDuration;
    void Awake()
    {
        boat = GetComponent<BoatDynamics2D>(); mission = GetComponent<MissionManager>();
        player = GetComponent<BoatActionSequence>(); bridge = GetComponent<BoatUdpBridge>();
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
        if (boat.Mode != BoatDynamics2D.ControlMode.Jev || mission.Progress == null || mission.Progress.Completed) return;
        Invalidate(); JevStarted = true; Pump();
    }
    public void StopJev() { JevStarted = false; Invalidate(); }
    public void Retry() { Invalidate(); }
    public void Invalidate()
    {
        player.Cancel(); bridge?.SetDecisionRequest(null);
        DecisionId = null; identity = null; LastError = null; State = RunState.Initializing;
    }
    private string Identity() => boat.SessionId + "/" + boat.EpisodeId + "/" + boat.RunId + "/" + mission.Revision;
    private string Parameters() => JsonUtility.ToJson(BoatParameters.Capture(boat)) + "/" + actionCount + "/" + actionDuration.ToString("R", System.Globalization.CultureInfo.InvariantCulture);
    void Update() { Pump(); }
    void FixedUpdate() { Advance(Time.fixedDeltaTime); }
    public void Pump()
    {
        bridge?.PumpIncoming();
        if (mission.Progress == null) { State = RunState.Initializing; return; }
        if (mission.Progress.Completed)
        { if (State != RunState.Completed) { player.Cancel(); bridge?.SetDecisionRequest(null); State = RunState.Completed; } return; }
        if (boat.Mode != BoatDynamics2D.ControlMode.Jev)
        { State = RunState.Manual; return; }
        if (!JevStarted) { State = RunState.Ready; return; }
        if (State == RunState.Failed) return;
        if (identity != null && (identity != Identity() || parameters != Parameters())) Invalidate();
        if (!player.HasPlan && State != RunState.Waiting) BeginDecision();
    }
    private void BeginDecision()
    {
        try
        {
            if (bridge == null || !bridge.Listening) throw new InvalidOperationException("UDP bridge unavailable.");
            DiscreteActionPlan.Steps(actionDuration, Time.fixedDeltaTime);
            requestedCount = actionCount;
            if (requestedCount < 1 || requestedCount > 10) throw new ArgumentException("Expected 1..10 actions per decision.");
            requestedDuration = actionDuration; requestTick = boat.Tick;
            identity = Identity(); parameters = Parameters(); DecisionId = Guid.NewGuid().ToString("N");
            var observation = ObservationBuilder.ForModel(boat, mission, Time.fixedDeltaTime, requestedCount, requestedDuration);
            var packet = Envelope("decision_request"); packet["observation"] = observation;
            bridge.SetDecisionRequest(packet.ToString(Newtonsoft.Json.Formatting.None));
            State = RunState.Waiting;
        }
        catch (Exception error) { Fail(error.Message); }
    }
    private JObject Envelope(string type) => new JObject {
        ["type"] = type, ["version"] = 4, ["session_id"] = boat.SessionId, ["episode_id"] = boat.EpisodeId,
        ["run_id"] = boat.RunId, ["decision_id"] = DecisionId, ["tick"] = requestTick };
    public void ReceiveDecision(JObject packet)
    {
        if (boat.Mode != BoatDynamics2D.ControlMode.Jev || DecisionId == null || identity != Identity()
            || (int?)packet["version"] != 4 || (string)packet["session_id"] != boat.SessionId
            || (string)packet["episode_id"] != boat.EpisodeId || (string)packet["run_id"] != boat.RunId
            || (string)packet["decision_id"] != DecisionId || (long?)packet["tick"] != requestTick) return;
        if (State == RunState.Executing || State == RunState.Completed || State == RunState.Failed)
        { bridge.SendPacket(Envelope("decision_ack").ToString(Newtonsoft.Json.Formatting.None)); return; }
        if (State != RunState.Waiting) return;
        try
        {
            if (parameters != Parameters()) { Invalidate(); return; }
            if ((string)packet["type"] == "decision_error") throw new InvalidOperationException((string)packet["error"] ?? "Python decision failed.");
            if ((string)packet["type"] != "decision_result") return;
            if (boat.Tick != requestTick) throw new InvalidOperationException("Simulation moved while waiting.");
            if (packet["actions"] is not JArray array || array.Count != requestedCount)
                throw new FormatException("Action count does not match the requested period.");
            foreach (var item in array)
                if (item?["acceleration_index"]?.Type != JTokenType.Integer || item?["turn_index"]?.Type != JTokenType.Integer)
                    throw new FormatException("Both action indices must be explicit integers.");
            if (packet["action_duration_s"]?.Type != JTokenType.Float && packet["action_duration_s"]?.Type != JTokenType.Integer)
                throw new FormatException("Missing action duration.");
            if (Math.Abs((double)packet["action_duration_s"] - requestedDuration) > 1e-6)
                throw new FormatException("Response changed action duration.");
            player.AcceptPlan(array.ToObject<DiscreteBoatAction[]>(), requestedDuration, Time.fixedDeltaTime);
            bridge.SetDecisionRequest(null); State = RunState.Executing;
            bridge.SendPacket(Envelope("decision_ack").ToString(Newtonsoft.Json.Formatting.None));
        }
        catch (Exception error) { Fail(error.Message); bridge?.SendPacket(Envelope("decision_ack").ToString(Newtonsoft.Json.Formatting.None)); }
    }
    private void Fail(string error)
    { player.Cancel(); bridge?.SetDecisionRequest(null); LastError = error; State = RunState.Failed; }
    public bool Advance(float dt)
    {
        Pump();
        if (mission.Progress == null || mission.Progress.Completed || State == RunState.Failed) return false;
        if (boat.Mode == BoatDynamics2D.ControlMode.Jev)
        {
            try { if (!player.PrepareStep(dt)) return false; }
            catch (Exception error) { Fail(error.Message); return false; }
        }
        Vector2 previous = boat.Position; boat.Step(dt); mission.Evaluate(previous);
        if (boat.Mode == BoatDynamics2D.ControlMode.Jev)
        { player.FinishStep(); if (!player.HasPlan) State = RunState.Initializing; }
        if (mission.Progress.Completed) { player.Cancel(); bridge?.SetDecisionRequest(null); State = RunState.Completed; }
        StepCompleted?.Invoke();
        return true;
    }
}
