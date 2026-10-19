"""Audit pinned Kev encoding/decoding and compare HTTP, native prefix and whole-row inference."""
import argparse
import json
from pathlib import Path

import torch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deployment", type=Path, default=Path("D:/AI/Kev-0.8B"))
    parser.add_argument("--factors", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a new output path")
    from kev.api import SystemOneRequest, to_answers, to_record
    from kev.checkpoint import Checkpoint, LoadOptions
    from kev.model import SPECIAL, admit, user_tokens
    manifest = json.loads((args.deployment / "models.json").read_text(encoding="utf-8-sig"))
    factors = json.loads(args.factors.read_text(encoding="utf-8"))["results"]
    root = Path(__file__).resolve().parents[1]
    worker = json.loads((root / "Logs/model-verification-20261018-kev/worker.jsonl").read_text(encoding="utf-8").splitlines()[0])
    # Whole-row inference of the 7.5k-token production payload exceeded the comfortable
    # memory budget on this 6 GB GPU. Keep parity checks to bounded short requests.
    tests = []
    for name, count, style, padding, group in (("numeric-3", 3, "numeric", 0, "count-label"),
                                               ("neutral-56", 56, "neutral", 0, "count-label"),
                                               ("numeric-56", 56, "numeric", 0, "count-label"),
                                               ("padded-14", 14, "neutral", 10, "length")):
        row = next(r for r in factors if r["count"] == count and r["style"] == style and r["padding"] == padding and r["group"] == group)
        tests.append((name, row["payload"], row["raw_response"]))
    keys = list(worker["payload"]["questions"]["action_0"]["criteria"])
    mappings = 0
    for order in (keys, keys[::-1], keys[17:]+keys[:17]):
        for i, expected in enumerate(order):
            p = [0.0]*56
            p[i] = 1.0
            answer = to_answers([p], [{"id": "action_0", "type": "choice", "keys": order}])
            assert answer["action_0"]["choice"] == expected
            mappings += 1
    ck = Checkpoint(manifest["kev_path"])
    opts = LoadOptions.from_env()
    assert opts.dtype == torch.bfloat16 and opts.backend == "torch" and not opts.cuda_graphs and not opts.fused
    tok, model = ck.load("cuda", opts)
    assert str(model.device).startswith("cuda") and not model.training
    print(json.dumps({"event": "loaded", "device": str(model.device), "dtype": str(model.dtype)}), flush=True)
    report = {"model_path": manifest["kev_path"], "model_revision": manifest["kev_revision"],
              "base_revision": ck.meta.base_revision, "option_isolation": ck.meta.option_isolation,
              "adapter_merged": opts.merge, "long_production_request_skipped": True,
              "device": str(model.device), "dtype": str(model.dtype), "known_logit_mappings": mappings, "cases": []}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for name, payload, served in tests:
        print(json.dumps({"event": "case_started", "case": name}), flush=True)
        record, meta = to_record(SystemOneRequest(**payload))
        enc = admit(model, tok, record, truncate=False)
        n_state = enc["seg"].count(0)
        assert enc["ids"][:n_state] == [tok.convert_tokens_to_ids(SPECIAL[0])] + user_tokens(tok, record["state"])
        for i, option in enumerate(record["questions"][0]["options"]):
            actual = [token for token, seg, slot in zip(enc["ids"], enc["seg"], enc["opt"]) if seg == 1 and slot == i]
            expected = [tok.convert_tokens_to_ids(SPECIAL[2])] + user_tokens(tok, option) + [tok.convert_tokens_to_ids(SPECIAL[3])]
            assert actual == expected
        ps = [p.tolist() for p in model.probs(enc)]
        native = to_answers(ps, meta)
        with torch.no_grad():
            full_ps = [torch.softmax(z, -1).float().cpu().tolist() for z in model.forward(enc)]
        full = to_answers(full_ps, meta)
        a, b, c = native["action_0"], served["answers"]["action_0"], full["action_0"]
        difference = max(abs(p-b["probabilities"][key]) for key, p in a["probabilities"].items())
        full_difference = max(abs(p-c["probabilities"][key]) for key, p in a["probabilities"].items())
        item = {"case": name, "payload": payload, "native": native, "http": served, "whole_row": full,
                "matches_http": a["choice"] == b["choice"] and difference <= .001,
                "max_http_probability_difference": difference,
                "prefix_matches_whole_row_choice": a["choice"] == c["choice"],
                "max_prefix_whole_row_probability_difference": full_difference,
                "tokens": len(enc["ids"]), "state_tokens": n_state, "options": len(record["questions"][0]["options"]),
                "complete_state_and_options": True, "state_truncated": enc["state_truncated"]}
        report["cases"].append(item)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({k: item[k] for k in ("case", "matches_http", "max_http_probability_difference", "prefix_matches_whole_row_choice", "max_prefix_whole_row_probability_difference")}), flush=True)
    report["http_parity_passed"] = all(r["matches_http"] for r in report["cases"])
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0 if report["http_parity_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
