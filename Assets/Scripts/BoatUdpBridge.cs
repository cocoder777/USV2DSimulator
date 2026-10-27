using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using Newtonsoft.Json.Linq;

[DefaultExecutionOrder(-100)]
[RequireComponent(typeof(BoatDynamics2D))]
public class BoatUdpBridge : MonoBehaviour
{
    public int port = 5005;
    public string targetAddress = "127.0.0.1";
    public int targetPort = 5006;
    public string situationGroup = "239.255.0.10";
    public int situationPort = 18010;
    public string multicastInterface = "127.0.0.1";
    public int AlgorithmStatus { get; private set; }
    public string AlgorithmDescription { get; private set; }
    public bool Listening => listening;
    public string CommunicationStatus => transportError ?? (!listening ? "Starting / stopped" : "Listening");
    UdpClient socket;
    Thread worker;
    volatile bool stopping, listening;
    volatile string transportError;
    readonly object gate = new object();
    readonly Queue<JObject> incoming = new();
    readonly Queue<(string type, List<(byte[] data, bool multicast)> frames)> outgoing = new();
    string statusIdentity;
    public event Action<JObject> DecisionReceived;

    void OnEnable()
    {
        stopping = false; transportError = null;
        try
        {
            if (port < 1 || port > 65535 || targetPort < 1 || targetPort > 65535 || situationPort < 1 || situationPort > 65535)
                throw new ArgumentException("UDP ports must be 1..65535.");
            var target = new IPEndPoint(IPAddress.Parse(targetAddress), targetPort);
            var group = IPAddress.Parse(situationGroup);
            if (target.Address.AddressFamily != AddressFamily.InterNetwork || group.GetAddressBytes()[0] < 224 || group.GetAddressBytes()[0] > 239)
                throw new ArgumentException("Expected IPv4 peer and multicast group.");
            var bound = new UdpClient(AddressFamily.InterNetwork);
            lock (gate) socket = bound;
            bound.ExclusiveAddressUse = true;
            bound.Client.Bind(new IPEndPoint(IPAddress.Any, port));
            bound.Client.SendTimeout = 100;
            bound.MulticastLoopback = true; bound.Ttl = 1;
            bound.Client.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.MulticastInterface, IPAddress.Parse(multicastInterface).GetAddressBytes());
            if (Environment.OSVersion.Platform == PlatformID.Win32NT)
                bound.Client.IOControl((IOControlCode)(-1744830452), new byte[] { 0 }, null);
            listening = true;
            var multicast = new IPEndPoint(group, situationPort);
            worker = new Thread(() => NetworkLoop(bound, target, multicast)) { IsBackground = true, Name = "USV UDP bridge" };
            worker.Start();
        }
        catch (Exception e) { listening = false; transportError = "UDP error: " + e.Message; lock (gate) { socket?.Close(); socket = null; } }
    }
    public void PumpIncoming()
    {
        for (int i = 0; i < 64; i++)
        {
            JObject packet;
            lock (gate) { if (incoming.Count == 0) break; packet = incoming.Dequeue(); }
            if (!listening) continue;
            var boat = GetComponent<BoatDynamics2D>();
            if ((string)packet["episode_id"] != boat.EpisodeId || (string)packet["run_id"] != boat.RunId) continue;
            var runner = GetComponent<SimulationRunner>();
            if (packet["decision_id"]?.Type == JTokenType.String && (string)packet["decision_id"] != runner.DecisionId) continue;
            if (packet["status"] != null && packet["tick"]?.Type == JTokenType.Integer && (long)packet["tick"] != boat.Tick) continue;
            if (packet["status"] != null && (GetComponent<MissionManager>().Failed ||
                (runner.State == SimulationRunner.RunState.Failed && (int)packet["status"] != 3) ||
                ((runner.State == SimulationRunner.RunState.Completed || runner.State == SimulationRunner.RunState.Stopped) && (int)packet["status"] != 2))) continue;
            if (packet["status"] != null)
            { AlgorithmStatus = (int)packet["status"]; AlgorithmDescription = (string)packet["description"]; }
            if (packet["type"] != null) DecisionReceived?.Invoke(packet);
        }
    }
    public void QueuePacket(JObject packet)
    {
        string type = (string)packet["type"];
        ushort kind = type == "stop_request" ? ClientProtocol.Stop : ClientProtocol.Task;
        uint stamp = ClientProtocol.Timestamp();
        JObject obs = packet["observation"] as JObject;
        var local = obs?["unity_only"]?.DeepClone() as JObject ?? new JObject();
        foreach (string field in new[] { "decision_id", "tick" })
            if (packet[field] != null) local[field] = packet[field].DeepClone();
        local["event"] = type;
        var content = new JObject { ["simId"] = packet["episode_id"]?.DeepClone(),
            ["id"] = packet["run_id"]?.DeepClone(), ["code"] = GetComponent<MissionManager>().TaskCode,
            ["params"] = obs?["params"]?.DeepClone() ?? packet["params"]?.DeepClone() ?? new JObject { ["platCode"] = 1001 }, ["unity_only"] = local };
        string taskIdentity = (string)content["simId"] + "/" + (string)content["id"];
        if (statusIdentity != taskIdentity || type == "decision_request" || type == "stop_request")
        { statusIdentity = taskIdentity; AlgorithmStatus = 0; AlgorithmDescription = null; }
        if (kind == ClientProtocol.Stop)
        {
            content.Remove("params"); // Resolve the formal task context at the receiver.
        }
        var frames = new List<(byte[] data, bool multicast)>();
        if (obs != null)
        {
            uint situationStamp = NextSituationStamp();
            local["situation_timestamp"] = situationStamp;
            foreach (JObject record in (JArray)obs["situation"])
            {
                frames.Add((ClientProtocol.SituationPacket(record, situationStamp), true));
            }
        }
        foreach (var frame in ClientProtocol.JsonPackets(content, kind, stamp)) frames.Add((frame, false));
        lock (gate) outgoing.Enqueue((type, frames));
    }
    uint lastSituationStamp;
    uint NextSituationStamp()
    {
        uint value = ClientProtocol.Timestamp(false);
        if (value <= lastSituationStamp && lastSituationStamp - value < 1000) value = lastSituationStamp + 1;
        return lastSituationStamp = value;
    }
    public void CancelDecisionRequest()
    {
        lock (gate)
        {
            int count = outgoing.Count;
            for (int i = 0; i < count; i++)
            { var item = outgoing.Dequeue(); if (item.type != "decision_request" && item.type != "stop_request") outgoing.Enqueue(item); }
        }
    }
    void NetworkLoop(UdpClient bound, IPEndPoint peer, IPEndPoint multicast)
    {
        var reassembler = new ClientProtocol.Reassembler();
        var controls = new Dictionary<uint, JObject>();
        var statuses = new Dictionary<uint, JObject>();
        try
        {
            while (!stopping)
            {
                try
                {
                    for (int i = 0; i < 64 && bound.Client.Poll(0, SelectMode.SelectRead); i++)
                    {
                        IPEndPoint source = new IPEndPoint(IPAddress.Any, 0);
                        byte[] data = bound.Receive(ref source);
                        if (!source.Equals(peer)) continue;
                        try
                        {
                            ushort kind = ClientProtocol.ParseHeader(data);
                            if (kind == ClientProtocol.Control)
                            {
                                var control = ClientProtocol.DecodeControl(data);
                                controls[(uint)control["timestamp"]] = control;
                            }
                            else if (kind == ClientProtocol.Status)
                            {
                                var status = reassembler.Add(data, source.ToString());
                                if (status == null) continue;
                                if ((string)status["code"] is not ("trackingMode" or "targetFollowing")) throw new FormatException("Unexpected algorithm code.");
                                if ((int?)status["status"] is not (1 or 2 or 3)) throw new FormatException("Invalid algorithm status.");
                                var meta = status["params"]?["unity_only"] as JObject;
                                var result = StatusEnvelope(status, meta);
                                if (meta?["decision_id"] != null)
                                {
                                    if ((int)status["status"] == 3)
                                    { result["type"] = "decision_error"; result["error"] = status["description"]?.DeepClone(); Enqueue(result); }
                                    else if (meta["control_timestamp"] != null) statuses[(uint)meta["control_timestamp"]] = status;
                                }
                                var display = StatusEnvelope(status, meta);
                                display["status"] = status["status"].DeepClone(); display["description"] = status["description"]?.DeepClone();
                                Enqueue(display);
                            }
                            foreach (uint key in statuses.Keys.ToArray())
                            {
                                if (!controls.TryGetValue(key, out var control)) continue;
                                var status = statuses[key];
                                if ((uint)control["platCode"] != 1001) { statuses.Remove(key); controls.Remove(key); continue; }
                                var result = StatusEnvelope(status, (JObject)status["params"]["unity_only"]);
                                result["type"] = "decision_result";
                                // Apply the actual quantized wire command, never the unquantized model value.
                                result["action"] = new JObject { ["speed"] = control["speed"].DeepClone(), ["course"] = control["course"].DeepClone() };
                                Enqueue(result); statuses.Remove(key); controls.Remove(key);
                            }
                            while (controls.Count > 64) controls.Remove(controls.Keys.First());
                            while (statuses.Count > 64) statuses.Remove(statuses.Keys.First());
                        }
                        catch (Exception e) when (e is FormatException || e is ArgumentException || e is Newtonsoft.Json.JsonException
                            || e is InvalidCastException || e is OverflowException) { /* Invalid frames never actuate the boat. */ }
                    }
                    List<(byte[] data, bool multicast)> batch = null;
                    lock (gate) { if (outgoing.Count > 0) batch = outgoing.Dequeue().frames; }
                    if (batch != null) foreach (var frame in batch)
                        bound.Send(frame.data, frame.data.Length, frame.multicast ? multicast : peer);
                }
                catch (SocketException e) when (e.SocketErrorCode == SocketError.ConnectionReset || e.SocketErrorCode == SocketError.ConnectionRefused
                    || e.SocketErrorCode == SocketError.MessageSize) { }
                Thread.Sleep(5);
            }
        }
        catch (Exception e) { if (!stopping) transportError = "UDP error: " + e.Message; }
        finally { listening = false; bound.Close(); }
    }
    static JObject StatusEnvelope(JObject status, JObject meta) => new JObject {
        ["episode_id"] = status["simId"]?.DeepClone(), ["run_id"] = status["id"]?.DeepClone(),
        ["decision_id"] = meta?["decision_id"]?.DeepClone(), ["tick"] = meta?["tick"]?.DeepClone() };
    void Enqueue(JObject value) { lock (gate) { if (incoming.Count >= 64) incoming.Dequeue(); incoming.Enqueue(value); } }
    void OnDisable()
    {
        stopping = true; listening = false;
        lock (gate) socket?.Close();
        worker?.Join(); worker = null;
        lock (gate) { incoming.Clear(); outgoing.Clear(); socket = null; }
    }
}
