"""Local UDP v2 smoke client. Standard library only; does not call Jev."""
import argparse
import json
import math
import socket
import time

MAX_PACKET = 8192


def control(state, sequence, acceleration, angular_acceleration_deg):
    return {
        "type": "control", "version": 2, "run_id": state["run_id"],
        "control_epoch": state["control_epoch"], "sequence": sequence,
        "acceleration": acceleration, "angular_acceleration_deg": angular_acceleration_deg,
    }


class StateReceiver:
    """Accept only complete valid states in increasing order for this client lifetime."""
    def __init__(self):
        self.last_sequence = -1

    def accept(self, payload):
        if not payload or len(payload) > MAX_PACKET:
            return None
        try:
            state = json.loads(payload.decode("utf-8"))
            if not isinstance(state, dict) or state.get("type") != "state" or state.get("version") != 2:
                return None
            sequence = state.get("state_sequence")
            if type(sequence) is not int or sequence <= self.last_sequence:
                return None
            for field in ("run_id", "control_epoch"):
                if not isinstance(state.get(field), str) or not state[field]:
                    return None
            if state.get("mode") not in ("Manual", "External"):
                return None
            for field in ("x", "y", "speed", "heading_deg"):
                if type(state.get(field)) not in (int, float) or not math.isfinite(state[field]):
                    return None
            self.last_sequence = sequence
            return state
        except (ValueError, UnicodeError, OverflowError):
            return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="Unity UDP IPv4 address")
    parser.add_argument("--port", type=int, default=5005, help="Unity command port")
    parser.add_argument("--listen-host", default="127.0.0.1")
    parser.add_argument("--listen-port", type=int, default=5006, help="Local state port")
    parser.add_argument("--log", help="Optional JSONL state log")
    parser.add_argument("--phase-seconds", type=float, default=3.0)
    parser.add_argument("--state-timeout", type=float, default=5.0)
    args = parser.parse_args()
    for name, value in (("phase-seconds", args.phase_seconds), ("state-timeout", args.state_timeout)):
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name} must be finite and positive")
    for port in (args.port, args.listen_port):
        if not 1 <= port <= 65535:
            parser.error("ports must be between 1 and 65535")
    log = None
    try:
        endpoint = (socket.gethostbyname(args.host), args.port)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            # Windows must report an occupied local port rather than share it silently.
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            sock.bind((args.listen_host, args.listen_port))
            if hasattr(socket, "SIO_UDP_CONNRESET"):
                sock.ioctl(socket.SIO_UDP_CONNRESET, False)
            sock.settimeout(0.1)
            if args.log:
                log = open(args.log, "w", encoding="utf-8")
            receiver = StateReceiver()
            sequence = 0
            started = None
            run_id = epoch = None
            last_state = None
            last_valid = time.monotonic()
            last_display = 0.0
            print(f"UDP listening on {args.listen_host}:{args.listen_port}. "
                  "Start Unity and select External mode.", flush=True)
            try:
                while True:
                    if time.monotonic() - last_valid > args.state_timeout:
                        raise TimeoutError("No fresh valid Unity state received before timeout")
                    try:
                        payload, source = sock.recvfrom(MAX_PACKET + 1)
                    except socket.timeout:
                        continue
                    except OSError as error:
                        if getattr(error, "winerror", None) in (10040, 10054):
                            continue
                        raise
                    if source != endpoint:
                        continue
                    state = receiver.accept(payload)
                    if state is None:
                        continue
                    now = time.monotonic()
                    last_valid = now
                    last_state = state
                    if log:
                        log.write(json.dumps(state, ensure_ascii=False, allow_nan=False) + "\n")
                        log.flush()
                    if state["run_id"] != run_id:
                        run_id, started = state["run_id"], None
                    if state["control_epoch"] != epoch:
                        epoch, sequence = state["control_epoch"], 0
                    if state["mode"] != "External":
                        started = None
                        continue
                    if started is None:
                        started = now
                    phase = int((now - started) / args.phase_seconds)
                    if phase >= 3:
                        break
                    acceleration, angular = [(1.0, 0.0), (0.5, 20.0), (0.0, 0.0)][phase]
                    sequence += 1
                    packet = control(state, sequence, acceleration, angular)
                    sock.sendto(json.dumps(packet, allow_nan=False).encode("utf-8"), endpoint)
                    if now - last_display >= 0.5:
                        print(f"phase={phase + 1} xy=({state['x']:.2f},{state['y']:.2f}) "
                              f"speed={state['speed']:.2f} heading={state['heading_deg']:.1f}", flush=True)
                        last_display = now
            finally:
                if last_state and last_state["mode"] == "External":
                    try:
                        sock.sendto(json.dumps(control(last_state, sequence + 1, 0, 0)).encode(), endpoint)
                    except OSError:
                        pass
            print("Finished. Sent best-effort zero input; Unity also clears input on timeout.")
    except (OSError, ValueError) as error:
        parser.exit(1, f"UDP bridge error: {error}\nCheck Unity Play Mode and both fixed ports.\n")
    except KeyboardInterrupt:
        print("Interrupted; Unity clears external input after the command timeout.")
    finally:
        if log:
            log.close()


if __name__ == "__main__":
    main()
