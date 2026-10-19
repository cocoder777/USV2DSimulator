"""Compare the pinned native Laya Agent with HTTP and audit local decoder/encoding paths."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import urllib.request

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "jev_api"))
from choice_control import ACTION_CHOICES  # noqa: E402


def mapping_checks():
    from laya.agent import Agent
    # No weights or GPU here: known logits test slot-to-label mapping independently.
    agent = Agent.__new__(Agent)
    agent.temperature_by_options = {}
    agent.temperature = [1, 1, 1]
    agent.lang_temperatures = {}
    keys = list(ACTION_CHOICES)
    completed = 0
    for order in (keys, keys[::-1], keys[17:]+keys[:17]):
        for index, expected in enumerate(order):
            logits = np.full((1, 56), -20, dtype=np.float32)
            logits[0, index] = 20
            question = {"t": "choice", "ins": "test", "crit": {key: "" for key in order}}
            answer = agent._decode_answers(logits, np.array([[1, 0]], dtype=np.float32),
                                           [{"markers": list(range(56))}], ["action_0"], {"action_0": question}, 0)
            assert answer["action_0"]["choice"] == expected
            completed += 1
    return {"laya_known_logit_mappings": completed, "passed": True}


def encoding_check(agent, payload):
    from laya.common import build_head, encode_text, render_options, serialize_state
    ids = list(payload["questions"])
    internal = {qid: agent._to_internal(payload["questions"][qid]) for qid in ids}
    items = agent._encode_state(payload["state"], ids, internal, max_len=4096, head_max_len=3072)
    evidence = []
    for qid, item in zip(ids, items):
        q = internal[qid]
        head, markers, stats = build_head(agent.tok, q, 3072)
        options = render_options(q)
        assert len(markers) == len(options) == stats["options_distinct"]
        for i, text in enumerate(options):
            expected = encode_text(agent.tok, " "+text.replace(agent.tok.mask_token, " "), add_special_tokens=False)["input_ids"]
            end = markers[i+1] if i+1 < len(markers) else len(head)-1
            assert head[markers[i]+1:end] == expected
        state = encode_text(agent.tok, serialize_state(payload["state"]).replace(agent.tok.mask_token, " "), add_special_tokens=False)["input_ids"]
        assert item["ids"][len(head):-1] == state
        assert not item["state_stats"]["truncated"]
        evidence.append({"options": len(options), "sequence_tokens": len(item["ids"]),
                         "state_tokens": len(state), "option_and_state_tokens_match": True,
                         "sequence_sha256": hashlib.sha256(json.dumps(item["ids"]).encode()).hexdigest()})
    return evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deployment", type=Path, default=Path("D:/AI/Laya"))
    parser.add_argument("--factors", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new output path")
    from laya.agent import Agent
    manifest = json.loads((args.deployment / "models.json").read_text(encoding="utf-8-sig"))
    report = {"mapping": mapping_checks(), "cases": [], "model_id": manifest["model_id"], "revision": manifest["revision"]}
    factors = json.loads(args.factors.read_text(encoding="utf-8"))["results"]
    worker = json.loads((ROOT / "Logs/model-verification-20261018-laya/worker.jsonl").read_text(encoding="utf-8").splitlines()[0])
    label = json.loads((ROOT / "Logs/model-verification-20261018-laya/label-check.json").read_text(encoding="utf-8"))[0]
    tests = [("real-initial", worker["payload"]), ("label-56-original", label["payload"])]
    for name, group, count, padding in (("neutral-3", "count-label", 3, 0),
                                        ("neutral-56", "count-label", 56, 0),
                                        ("padded-14", "length", 14, 10)):
        row = next(r for r in factors if r["group"] == group and r["count"] == count and r["style"] == "neutral" and r["padding"] == padding)
        tests.append((name, row["payload"]))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with Agent(manifest["path"], device="cuda", backend="eager") as agent:
        assert agent.device.type == "cuda" and not agent.model.training
        report.update(device=str(agent.device), dtype=str(agent.dtype), module=sys.modules["laya.agent"].__file__,
                      checkpoint_max_len=agent.cfg.get("max_len"), checkpoint_head_max_len=agent.cfg.get("head_max_len"),
                      parallel_options=agent.parallel_options)
        for name, payload in tests:
            encoding = encoding_check(agent, payload)
            native = agent.predict(payload["state"], payload["questions"], max_len=4096, head_max_len=3072)
            request = urllib.request.Request("http://127.0.0.1:8010/v1/systemone", data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(request, timeout=60) as response:
                served = json.load(response)
            a = native["answers"]["action_0"]
            b = served["answers"]["action_0"]
            difference = max(abs(p-b["probabilities"][key]) for key, p in a["probabilities"].items())
            same = a["choice"] == b["choice"] and difference <= .001 and native["usage"]["input_tokens"] == served["usage"]["input_tokens"]
            report["cases"].append({"case": name, "matches_http": same, "max_probability_difference": difference,
                                    "encoding": encoding, "payload": payload, "native": native, "http": served})
            args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
            print(json.dumps({"case": name, "matches_http": same, "max_probability_difference": difference}), flush=True)
        report["cpu_fallback_count"] = agent.cpu_fallback_count
    report["passed"] = all(case["matches_http"] for case in report["cases"]) and report["cpu_fallback_count"] == 0
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
