using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Text;
using System.Threading;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

// Runs real Unity components and loopback UDP without saving scenes.
public static class BoatBridgeValidation
{
    private static void Invoke(object target, string method) => target.GetType()
        .GetMethod(method, BindingFlags.Instance | BindingFlags.NonPublic).Invoke(target, null);
    private static void Check(bool condition, string message)
    {
        if (!condition) throw new Exception(message);
    }
    private static UdpClient Bind(int port = 0)
    {
        var result = new UdpClient(AddressFamily.InterNetwork);
        result.ExclusiveAddressUse = true;
        result.Client.Bind(new IPEndPoint(IPAddress.Loopback, port));
        result.Client.ReceiveTimeout = 1000;
        if (Environment.OSVersion.Platform == PlatformID.Win32NT)
            result.Client.IOControl((IOControlCode)(-1744830452), new byte[] { 0 }, null);
        return result;
    }
    private static int Port(UdpClient client) => ((IPEndPoint)client.Client.LocalEndPoint).Port;
    private static string Command(string run, string epoch, long sequence, float accel = 1, float angular = 20)
    {
        return "{\"type\":\"control\",\"version\":2,\"run_id\":\"" + run +
            "\",\"control_epoch\":\"" + epoch + "\",\"sequence\":" + sequence +
            ",\"acceleration\":" + accel.ToString(CultureInfo.InvariantCulture) +
            ",\"angular_acceleration_deg\":" + angular.ToString(CultureInfo.InvariantCulture) + "}";
    }

    public static void Run()
    {
        GameObject root = null;
        BoatUdpBridge bridge = null;
        UdpClient client = null;
        try
        {
            EditorSceneManager.OpenScene("Assets/Scenes/Main.unity");
            var sceneBoat = GameObject.Find("Boat");
            Check(sceneBoat != null && sceneBoat.GetComponent<BoatControlInput>() != null
                && sceneBoat.GetComponent<BoatUdpBridge>() != null, "Scene components missing");
            Check(sceneBoat.GetComponent<BoatDynamics2D>().maxAcceleration == 1.5f,
                "Original scene parameter changed");
            root = new GameObject("UDP validation (temporary)");
            var boat = root.AddComponent<BoatDynamics2D>();
            Invoke(boat, "Awake");
            boat.SetControl(100, 1000);
            Check(boat.AccelerationCommandValue == 1.5f &&
                Mathf.Abs(boat.AngularAccelerationCommandDeg - 60) < 0.001f, "Clamp failed");
            boat.SetControl(float.NaN, 0);
            Check(boat.AccelerationCommandValue == 1.5f, "Invalid input changed control");
            boat.ResetBoat();
            bridge = root.AddComponent<BoatUdpBridge>();
            Invoke(bridge, "Awake");
            client = Bind();
            bridge.targetPort = Port(client);
            using (var reservation = Bind()) bridge.port = Port(reservation);
            Invoke(bridge, "OnEnable");
            Action<int> pump = milliseconds => {
                var watch = System.Diagnostics.Stopwatch.StartNew();
                do {
                    Invoke(bridge, "FixedUpdate"); Invoke(boat, "FixedUpdate");
                    Invoke(bridge, "LateUpdate"); Thread.Sleep(5);
                } while (watch.ElapsedMilliseconds < milliseconds);
            };
            var endpoint = new IPEndPoint(IPAddress.Loopback, bridge.port);
            Action<string> send = json => {
                byte[] bytes = Encoding.UTF8.GetBytes(json);
                client.Send(bytes, bytes.Length, endpoint); pump(30);
            };
            Func<BoatStateSnapshot> latest = () => {
                pump(120);
                BoatStateSnapshot state = null;
                while (client.Available > 0) {
                    var source = new IPEndPoint(IPAddress.Any, 0);
                    byte[] bytes = client.Receive(ref source);
                    Check(source.Equals(endpoint), "State source differs from configured Unity port");
                    state = JsonUtility.FromJson<BoatStateSnapshot>(Encoding.UTF8.GetString(bytes));
                }
                Check(state != null && state.version == 2 && state.type == "state", "No UDP state");
                return state;
            };
            var snapshot = latest();
            Check(bridge.Listening && snapshot.state_sequence > 0, "Bridge not listening");
            Check(latest().state_sequence > snapshot.state_sequence, "State sequence did not increase");
            boat.SetControl(0.25f, 0);
            send(Command(boat.RunId, bridge.ControlEpoch, 1));
            Check(boat.AccelerationCommandValue == 0.25f, "Manual input overridden");
            boat.SetMode(BoatDynamics2D.ControlMode.External); pump(10);
            send(Command(boat.RunId, bridge.ControlEpoch, 5));
            Check(bridge.ControlActive && boat.AccelerationCommandValue == 1 &&
                boat.Speed > 0 && boat.YawRateDeg > 0, "External motion failed");
            send(Command(boat.RunId, bridge.ControlEpoch, 5, -1));
            send(Command(boat.RunId, bridge.ControlEpoch, 4, -1));
            send(Command("old-run", bridge.ControlEpoch, 6, -1));
            send(Command(boat.RunId, "old-epoch", 6, -1));
            send("{broken json");
            send("{\"type\":\"control\",\"version\":2,\"run_id\":\"" + boat.RunId +
                "\",\"control_epoch\":\"" + bridge.ControlEpoch + "\",\"sequence\":6}");
            send(Command(boat.RunId, bridge.ControlEpoch, 6).Replace("\"version\":2", "\"version\":1"));
            send(Command(boat.RunId, bridge.ControlEpoch, 6).Replace("\"acceleration\":1", "\"acceleration\":1e999"));
            send(new string(' ', 8193));
            using (var stranger = Bind()) {
                byte[] bytes = Encoding.UTF8.GetBytes(Command(boat.RunId, bridge.ControlEpoch, 99, -1));
                stranger.Send(bytes, bytes.Length, endpoint); pump(20);
            }
            Check(boat.AccelerationCommandValue == 1, "Invalid/reordered/source-mismatched packet accepted");
            string expired = bridge.ControlEpoch;
            pump(600);
            Check(bridge.TimedOut && boat.AccelerationCommandValue == 0 &&
                expired != bridge.ControlEpoch, "Timeout did not invalidate epoch");
            send(Command(boat.RunId, expired, 100));
            Check(boat.AccelerationCommandValue == 0, "Expired epoch revived control");
            send(Command(boat.RunId, bridge.ControlEpoch, 1));
            Check(bridge.ControlActive && boat.AccelerationCommandValue == 1, "Timeout recovery failed");
            boat.maxAcceleration = 0.2f; pump(10);
            Check(boat.AccelerationCommandValue == 0.2f, "Held command not re-clamped");
            string oldRun = boat.RunId, oldEpoch = bridge.ControlEpoch;
            boat.ResetBoat(); pump(10);
            send(Command(oldRun, oldEpoch, 2));
            Check(boat.AccelerationCommandValue == 0 && oldEpoch != bridge.ControlEpoch,
                "Reset did not invalidate controls");
            send(Command(boat.RunId, bridge.ControlEpoch, 1, 0.1f, 0));
            int clientPort = Port(client);
            client.Close(); pump(600);
            Check(boat.AccelerationCommandValue == 0, "Silent client exit did not time out");
            client = Bind(clientPort);
            snapshot = latest();
            send(Command(snapshot.run_id, snapshot.control_epoch, 0, 0.1f, 0));
            Check(bridge.ControlActive && boat.AccelerationCommandValue == 0.1f, "Client restart failed");
            boat.SetMode(BoatDynamics2D.ControlMode.Manual); pump(10);
            Check(boat.AccelerationCommandValue == 0, "Mode switch retained input");
            Invoke(bridge, "OnDisable");
            using (var occupied = Bind(bridge.port)) {
                Invoke(bridge, "OnEnable");
                Check(!bridge.Listening && bridge.CommunicationStatus.StartsWith("UDP error:"),
                    "Occupied UDP port not reported");
                Invoke(bridge, "OnDisable");
            }
            Invoke(bridge, "OnEnable");
            Check(bridge.Listening, "Port could not be reused after shutdown");
            Invoke(bridge, "OnDisable"); bridge = null;
            client.Close();
            UnityEngine.Object.DestroyImmediate(root); root = null;
            Debug.Log("BOAT_BRIDGE_VALIDATION_PASS: UDP state/control, scene, manual isolation, motion, limits, invalid/duplicate/reordered/stale/source packets, epoch expiry, timeout/recovery, reset, client restart, port conflict/reuse.");
            EditorApplication.Exit(0);
        }
        catch (Exception exception)
        {
            Debug.LogException(exception);
            client?.Close();
            if (bridge != null) Invoke(bridge, "OnDisable");
            if (root != null) UnityEngine.Object.DestroyImmediate(root);
            EditorApplication.Exit(1);
        }
    }
}
