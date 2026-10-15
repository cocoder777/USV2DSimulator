"""UDP client tests with real local datagrams; does not replace Unity validation."""
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest

from boat_client import StateReceiver

CLIENT = Path(__file__).with_name("boat_client.py")


def snapshot(sequence, run="first", epoch="epoch1"):
    return {"type": "state", "version": 2, "state_sequence": sequence,
            "run_id": run, "control_epoch": epoch, "mode": "External",
            "x": 1, "y": 2, "speed": 1, "heading_deg": 10}


class BoatClientTests(unittest.TestCase):
    def test_state_validation_and_ordering(self):
        receiver = StateReceiver()
        encode = lambda value: json.dumps(value).encode()
        self.assertIsNotNone(receiver.accept(encode(snapshot(2))))
        for value in (snapshot(2), snapshot(1), {"type": "state"}, [], None):
            self.assertIsNone(receiver.accept(encode(value)))
        for packet in (b"{broken", b"\xff", b" " * 8193):
            self.assertIsNone(receiver.accept(packet))
        invalid = snapshot(3)
        invalid["speed"] = float("nan")
        self.assertIsNone(receiver.accept(encode(invalid)))
        self.assertIsNotNone(receiver.accept(encode(snapshot(3, "second", "epoch2"))))

    def test_phases_epochs_runs_and_log(self):
        commands, failures = [], []
        done = threading.Event()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation, \
                tempfile.TemporaryDirectory() as directory:
            server.bind(("127.0.0.1", 0))
            reservation.bind(("127.0.0.1", 0))
            port = server.getsockname()[1]
            client_port = reservation.getsockname()[1]
            reservation.close()
            server.settimeout(0.01)

            def serve():
                try:
                    index = 0
                    while not done.is_set():
                        # Change run and epoch once; duplicates and old states must not trigger commands.
                        state = snapshot(index, "first" if index < 30 else "second",
                                         "epoch1" if index < 60 else "epoch2")
                        server.sendto(json.dumps(state).encode(), ("127.0.0.1", client_port))
                        server.sendto(json.dumps(state).encode(), ("127.0.0.1", client_port))
                        if index:
                            server.sendto(json.dumps(snapshot(index - 1)).encode(), ("127.0.0.1", client_port))
                        server.sendto(b"{invalid", ("127.0.0.1", client_port))
                        try:
                            packet, source = server.recvfrom(8192)
                            commands.append(json.loads(packet))
                            self.assertEqual(source[1], client_port)
                        except (socket.timeout, ConnectionResetError):
                            pass
                        # Keep the first state until the subprocess is ready; startup time
                        # must not consume the simulated run/epoch transitions.
                        if commands:
                            index += 1
                        time.sleep(0.005)
                except Exception as error:
                    failures.append(error)

            thread = threading.Thread(target=serve)
            thread.start()
            try:
                log = Path(directory) / "states.jsonl"
                result = subprocess.run(
                    [sys.executable, str(CLIENT), "--port", str(port), "--listen-port", str(client_port),
                     "--phase-seconds", "0.3", "--state-timeout", "2", "--log", str(log)],
                    capture_output=True, text=True, timeout=8)
                self.assertEqual(result.returncode, 0, result.stderr)
                states = [json.loads(line) for line in log.read_text().splitlines()]
                numbers = [state["state_sequence"] for state in states]
                self.assertEqual(numbers, sorted(set(numbers)))
            finally:
                done.set()
                thread.join(3)
            self.assertFalse(thread.is_alive())
            self.assertEqual(failures, [])
        outputs = {(message["acceleration"], message["angular_acceleration_deg"]) for message in commands}
        self.assertTrue({(1, 0), (0.5, 20), (0, 0)} <= outputs, outputs)
        for epoch in ("epoch1", "epoch2"):
            numbers = [message["sequence"] for message in commands if message["control_epoch"] == epoch]
            self.assertTrue(numbers, f"No commands in {epoch}")
            self.assertEqual(numbers[0], 1)
            self.assertEqual(numbers, sorted(set(numbers)))
        self.assertTrue(any(message["run_id"] == "second" for message in commands))

    def test_no_states_timeout_and_port_reuse(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(("127.0.0.1", 0))
            port = reservation.getsockname()[1]
        result = subprocess.run(
            [sys.executable, str(CLIENT), "--listen-port", str(port), "--state-timeout", "0.2"],
            capture_output=True, text=True, timeout=4)
        self.assertEqual(result.returncode, 1)
        self.assertIn("No fresh valid Unity state", result.stderr)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reused:
            reused.bind(("127.0.0.1", port))

    def test_occupied_port_reports_error(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                reservation.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            reservation.bind(("127.0.0.1", 0))
            result = subprocess.run(
                [sys.executable, str(CLIENT), "--listen-port", str(reservation.getsockname()[1])],
                capture_output=True, text=True, timeout=4)
        self.assertEqual(result.returncode, 1)
        self.assertIn("UDP bridge error:", result.stderr)


if __name__ == "__main__":
    unittest.main()
