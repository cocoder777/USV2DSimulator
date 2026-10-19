using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;
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
    [Min(1f)] public float stateFrequency = 10f;
    public bool Listening => listening;
    public string CommunicationStatus => transportError ?? (!listening ? "Starting / stopped" : "Listening");

    private BoatDynamics2D boat;
    private UdpClient socket;
    private Thread worker;
    private volatile bool stopping, listening;
    private volatile string transportError;
    private readonly object gate = new object();
    private readonly Queue<string> incoming = new Queue<string>();
    private byte[] outgoing;
    private byte[] pendingDecision;
    public event Action<JObject> DecisionReceived;
    private long outgoingRevision, stateSequence;
    private double nextStateTime;
    private static double Now => (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;

    void Awake() { boat = GetComponent<BoatDynamics2D>(); }

    void OnEnable()
    {
        stopping = false;
        transportError = null;
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

    public void PumpIncoming()
    {
        for (int i = 0; i < 64; i++)
        {
            string message;
            lock (gate) {
                if (incoming.Count == 0) break;
                message = incoming.Dequeue();
            }
            if (!listening || boat.Mode != BoatDynamics2D.ControlMode.Jev) continue;
            try
            {
                var packet = JObject.Parse(message);
                if ((string)packet["type"] == "decision_result" || (string)packet["type"] == "decision_error")
                    DecisionReceived?.Invoke(packet);
            }
            catch (Exception exception) when (exception is ArgumentException || exception is Newtonsoft.Json.JsonException
                || exception is InvalidCastException || exception is FormatException || exception is OverflowException)
            { /* Invalid datagrams never change the simulation. */ }
        }
    }

    public void QueueDecisionRequest(string json)
    {
        byte[] bytes = Encoding.UTF8.GetBytes(json);
        if (bytes.Length > 8192) throw new ArgumentException("Decision request exceeds 8192-byte protocol budget.");
        lock (gate) { pendingDecision = bytes; }
    }

    public void CancelDecisionRequest() { lock (gate) { pendingDecision = null; } }

    void LateUpdate()
    {
        if (Now < nextStateTime) return;
        nextStateTime = Now + 1.0 / Math.Max(1f, stateFrequency);
        var state = BoatStateSnapshot.Capture(boat);
        state.state_sequence = ++stateSequence;
        byte[] bytes = Encoding.UTF8.GetBytes(state.ToJson());
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
                            incoming.Enqueue(json);
                        }
                    }
                    byte[] snapshot; long revision;
                    lock (gate) { snapshot = outgoing; revision = outgoingRevision; }
                    if (snapshot != null && revision != sentRevision)
                    {
                        if (snapshot.Length <= 8192) boundSocket.Send(snapshot, snapshot.Length, endpoint);
                        sentRevision = revision;
                    }
                    // 取走即清空，每份决策只发送一次；新决策由 SimulationRunner 显式提交。
                    byte[] decision;
                    lock (gate) { decision = pendingDecision; pendingDecision = null; }
                    if (decision != null) boundSocket.Send(decision, decision.Length, endpoint);
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
        lock (gate) { incoming.Clear(); outgoing = null; pendingDecision = null; socket = null; }
    }
}
