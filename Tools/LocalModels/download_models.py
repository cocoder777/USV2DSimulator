"""Download immutable pinned snapshots into this deployment and record SHA-256 hashes."""
import hashlib
import json
from pathlib import Path

from huggingface_hub import snapshot_download

ROOT = Path(__file__).resolve().parent


def download(root=ROOT):
    pin = json.loads((root / "pins.json").read_text(encoding="utf-8"))
    manifest_file = root / "models.json"
    if manifest_file.exists():
        raise FileExistsError("models.json already exists; refusing to replace an existing deployment")
    kev = "base_id" in pin
    patterns = (["*.json", "*.safetensors", "*.pt", "*.txt", "*.jinja", "*.model", "README.md", "LICENSE*"]
                if kev else ["rl_agent_config.json", "model.safetensors", "tokenizer/*", "encoder/*", "README.md", "LICENSE*"])
    model = Path(snapshot_download(pin["model_id"], revision=pin["revision"],
                                  allow_patterns=patterns, max_workers=2))
    manifest = {"source_commit": pin["source_commit"], "files": {}}
    paths = [model]
    if kev:
        base = Path(snapshot_download(pin["base_id"], revision=pin["base_revision"],
                                     allow_patterns=patterns, max_workers=2))
        paths.append(base)
        adapter = json.loads((model / "adapter_config.json").read_text())
        if adapter.get("base_model_name_or_path") != pin["base_id"]:
            raise ValueError("Kev adapter base identity differs from the pinned base")
        import torch
        head = torch.load(model / "head.pt", map_location="cpu", weights_only=True)
        if head.get("base") != pin["base_id"] or head.get("base_revision") != pin["base_revision"]:
            raise ValueError("Kev head base identity/revision differs from the pinned base")
        manifest.update(kev_id=pin["model_id"], kev_revision=pin["revision"], kev_path=str(model),
                        base_id=pin["base_id"], base_revision=pin["base_revision"], base_path=str(base))
    else:
        manifest.update(model_id=pin["model_id"], revision=pin["revision"], path=str(model))
        provenance = root / "provenance"
        provenance.mkdir(exist_ok=True)
        (provenance / "original-tokenizer-config.json").write_bytes((model / "tokenizer/tokenizer_config.json").read_bytes())
    for directory in paths:
        for path in sorted(directory.rglob("*")):
            if path.is_file():
                digest = hashlib.sha256()
                with path.open("rb") as stream:
                    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                        digest.update(chunk)
                name = str(path) if kev else str(path.relative_to(model))
                manifest["files"][name] = {"bytes": path.stat().st_size, "sha256": digest.hexdigest()}
    # Exclusive creation also prevents accidental manifest replacement on a second invocation.
    with manifest_file.open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2)
    print(json.dumps({key: value for key, value in manifest.items() if key != "files"}, indent=2))


if __name__ == "__main__":
    download()
