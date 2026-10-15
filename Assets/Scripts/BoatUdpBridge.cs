using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

[DefaultExecutionOrder(-100)]
[RequireComponent(typeof(BoatDynamics2D))]
public class BoatUdpBridge : MonoBehaviour
{
    public int port = 5005;
    public string targetAddress = "127.0.0.1";
    public int targetPort = 5006;
    [Min(1f)] public float stateFrequency = 10f;
    [Min(0.01f)] public float commandTimeout = 0.5f;
    public bool Listening => listening;
    public string ControlEpoch { get; private set; }
    public double CommandAge => lastCommandTime < 0 ? -1 : Now - lastCommandTime;
    public bool ControlActive => listening && boat != null
        && boat.Mode == BoatDynamics2D.ControlMode.External
        && lastCommandTime >= 0 && CommandAge <= TimeoutSeconds;
    public bool TimedOut => boat != null && boat.Mode == BoatDynamics2D.ControlMode.External && !ControlActive;
    public string CommunicationStatus => transportError ?? (!listening ? "Starting / stopped"
        : ControlActive ? "Control active" : TimedOut ? "Control timed out / waiting" : "Listening");
    private double TimeoutSeconds => Math.Max(0.01, commandTimeout);

    [Serializable]
    private class ControlMessage
    {
        public string type;
        public int version = -1;
        public string run_id;
        public string control_epoch;
        public long sequence = -1;
        public float acceleration = float.NaN;
        public float angular_acceleration_deg = float.NaN;
    }

    private struct Received { public string json; public double time; }
    private BoatDynamics2D boat;
    private UdpClient socket;
    private Thread worker;
    private volatile bool stopping, listening;
    private volatile string transportError;
    private readonly object gate = new object();
    private readonly Queue<Received> incoming = new Queue<Received>();
    private byte[] outgoing;
    private long outgoingRevision, lastSequence = -1, stateSequence;
    private string seenRun;
    private double lastCommandTime = -1, nextStateTime;
    private static double Now => (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;

    void Awake() { boat = GetComponent<BoatDynamics2D>(); }

    void OnEnable()
    {
        stopping = false;
        transportError = null;
        seenRun = boat.RunId;
        RotateEpoch();
        nextStateTime = 0;
        try
        {
            if (port < 1 || port > 65535 || targetPort < 1 || targetPort > 65535)
                throw new ArgumentException("UDP ports must be between 1 and 65535.");
            IPAddress target = IPAddress.Parse(targetAddress);
            if (target.AddressFamily != AddressFamily.InterNetwork || !IPAddress.IsLoopback(target))
                throw new ArgumentException("This bridge requires an IPv4 loopback target address.");
            var endpoint = new IPEndPoint(target, targetPort);
            var boundSocket = new UdpClient(AddressFamily.InterNetwork);
            lock (gate) { socket = boundSocket; }
            boundSocket.ExclusiveAddressUse = true;
            boundSocket.Client.Bind(new IPEndPoint(IPAddress.Loopback, port));
            boundSocket.Client.SendTimeout = 100;
            // A missing UDP receiver on Windows must not terminate our receive loop.
            if (Environment.OSVersion.Platform == PlatformID.Win32NT)
                boundSocket.Client.IOControl((IOControlCode)(-1744830452), new byte[] { 0 }, null);
            listening = true;
            worker = new Thread(() => NetworkLoop(boundSocket, endpoint)) {
                IsBackground = true, Name = "USV UDP bridge"
            };
            worker.Start();
        }
        catch (Exception exception)
        {
            listening = false;
            transportError = "UDP error: " + exception.Message;
            lock (gate) { socket?.Close(); socket = null; }
        }
    }

    private void RotateEpoch()
    {
        ControlEpoch = Guid.NewGuid().ToString("N");
        lastSequence = -1;
        lastCommandTime = -1;
        lock (gate) { incoming.Clear(); }
        if (boat.Mode == BoatDynamics2D.ControlMode.External) boat.ClearControl();
    }

    private void RefreshEpoch()
    {
        if (seenRun != boat.RunId || (lastCommandTime >= 0 && CommandAge > TimeoutSeconds))
        {
            seenRun = boat.RunId;
            RotateEpoch();
        }
    }

    void FixedUpdate()
    {
        // Expire the old epoch BEFORE inspecting queued commands, preventing late revival.
        RefreshEpoch();
        for (int i = 0; i < 64; i++)
        {
            Received message;
            lock (gate) {
                if (incoming.Count == 0) break;
                message = incoming.Dequeue();
            }
            if (!listening || Now - message.time > TimeoutSeconds
                || boat.Mode != BoatDynamics2D.ControlMode.External) continue;
            try
            {
                var control = new ControlMessage();
                JsonUtility.FromJsonOverwrite(message.json, control);
                if (control.type != "control" || control.version != 2 || control.run_id != boat.RunId
                    || control.control_epoch != ControlEpoch || control.sequence < 0
                    || control.sequence <= lastSequence || !BoatDynamics2D.IsFinite(control.acceleration)
                    || !BoatDynamics2D.IsFinite(control.angular_acceleration_deg)) continue;
                boat.SetControl(control.acceleration, control.angular_acceleration_deg);
                lastSequence = control.sequence;
                lastCommandTime = message.time;
            }
            catch (ArgumentException) { /* Invalid JSON does not refresh the control timeout. */ }
        }
        if (TimedOut) boat.ClearControl();
    }

    void LateUpdate()
    {
        RefreshEpoch();
        if (Now < nextStateTime) return;
        nextStateTime = Now + 1.0 / Math.Max(1f, stateFrequency);
        var state = BoatStateSnapshot.Capture(boat);
        state.state_sequence = ++stateSequence;
        state.control_epoch = ControlEpoch;
        state.control_active = ControlActive;
        state.command_timed_out = TimedOut;
        state.command_age_seconds = CommandAge;
        byte[] bytes = Encoding.UTF8.GetBytes(JsonUtility.ToJson(state));
        lock (gate) { outgoing = bytes; outgoingRevision++; }
    }

    // Socket I/O only; never call Unity APIs from the worker.
    private void NetworkLoop(UdpClient boundSocket, IPEndPoint endpoint)
    {
        long sentRevision = -1;
        var utf8 = new UTF8Encoding(false, true);
        try
        {
            while (!stopping)
            {
                try
                {
                    for (int i = 0; i < 64 && boundSocket.Client.Poll(0, SelectMode.SelectRead); i++)
                    {
                        IPEndPoint source = new IPEndPoint(IPAddress.Any, 0);
                        byte[] bytes = boundSocket.Receive(ref source);
                        if (!source.Equals(endpoint) || bytes.Length == 0 || bytes.Length > 8192) continue;
                        string json;
                        try { json = utf8.GetString(bytes); }
                        catch (DecoderFallbackException) { continue; }
                        lock (gate) {
                            if (incoming.Count >= 64) incoming.Dequeue();
                            incoming.Enqueue(new Received { json = json, time = Now });
                        }
                    }
                    byte[] snapshot; long revision;
                    lock (gate) { snapshot = outgoing; revision = outgoingRevision; }
                    if (snapshot != null && revision != sentRevision)
                    {
                        if (snapshot.Length <= 8192) boundSocket.Send(snapshot, snapshot.Length, endpoint);
                        sentRevision = revision;
                    }
                }
                catch (SocketException exception) when (exception.SocketErrorCode == SocketError.ConnectionReset
                    || exception.SocketErrorCode == SocketError.ConnectionRefused
                    || exception.SocketErrorCode == SocketError.MessageSize) { /* Drop this datagram. */ }
                Thread.Sleep(5);
            }
        }
        catch (Exception exception) { if (!stopping) transportError = "UDP error: " + exception.Message; }
        finally { listening = false; boundSocket.Close(); }
    }

    void OnDisable()
    {
        stopping = true;
        listening = false;
        lock (gate) { socket?.Close(); }
        // Socket closure unblocks receive; joining prevents a previous worker using new state on re-enable.
        worker?.Join();
        worker = null;
        lock (gate) { incoming.Clear(); outgoing = null; socket = null; }
        if (boat != null && boat.Mode == BoatDynamics2D.ControlMode.External) boat.ClearControl();
    }
}
