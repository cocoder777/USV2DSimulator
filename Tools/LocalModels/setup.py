"""Install pinned Windows CUDA deployments, or verify existing ones without writing them."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def run(command, **kwargs):
    return subprocess.check_output([str(value) for value in command], text=True,
                                   encoding="utf-8", errors="replace", **kwargs).strip()


def file_hash(path):
    with Path(path).open("rb") as stream:
        digest = hashlib.sha256()
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def normalized_tokenizer(original):
    expected = dict(original)
    if expected.get("tokenizer_class") in (None, "TokenizersBackend"):
        expected["tokenizer_class"] = "PreTrainedTokenizerFast"
        expected.pop("backend", None)
        expected.pop("is_local", None)
    if isinstance(expected.get("extra_special_tokens"), list):
        expected["extra_special_tokens"] = {
            f"extra_{index}": token for index, token in enumerate(expected["extra_special_tokens"])}
    return expected


def verify_manifest(directory, backend, pin):
    manifest = read_json(directory / "models.json")
    fields = {"source_commit": pin["source_commit"]}
    if backend == "laya":
        fields.update(model_id=pin["model_id"], revision=pin["revision"])
        roots = [Path(manifest["path"])]
        required = [roots[0] / name for name in (
            "model.safetensors", "rl_agent_config.json", "encoder/config.json",
            "tokenizer/tokenizer.json", "tokenizer/tokenizer_config.json")]
    else:
        fields.update(kev_id=pin["model_id"], kev_revision=pin["revision"],
                      base_id=pin["base_id"], base_revision=pin["base_revision"])
        roots = [Path(manifest["kev_path"]), Path(manifest["base_path"])]
        required = [roots[0] / "head.pt", roots[0] / "adapter_model.safetensors",
                    roots[0] / "adapter_config.json", roots[1] / "config.json"]
        if not list(roots[1].glob("*.safetensors")):
            raise ValueError("Missing Kev base weights")
    for field, value in fields.items():
        if manifest.get(field) != value:
            raise ValueError(f"Pinned {field} mismatch in {directory}")
    for root in roots:
        if not root.resolve().is_relative_to(directory.resolve()):
            raise ValueError(f"Model path is outside this deployment: {root}")
    records = manifest.get("files", {})
    if not records:
        raise ValueError("Missing model file checksum records")
    covered = set()
    for name, record in records.items():
        path = Path(name) if backend == "kev" else roots[0] / name
        resolved = path.resolve()
        if not any(resolved.is_relative_to(root.resolve()) for root in roots):
            raise ValueError(f"Checksum path is outside the model snapshots: {name}")
        covered.add(resolved)
        actual = file_hash(path)
        if actual == record["sha256"] and path.stat().st_size == record["bytes"]:
            continue
        # Official Laya loading normalizes tokenizer metadata, not weights.
        if backend == "laya" and path == roots[0] / "tokenizer/tokenizer_config.json":
            original = directory / "provenance/original-tokenizer-config.json"
            if original.is_file() and file_hash(original) == record["sha256"]:
                if read_json(path) == normalized_tokenizer(read_json(original)):
                    continue
        raise ValueError(f"Model checksum mismatch: {path}")
    if not all(path.is_file() and path.resolve() in covered for path in required):
        raise ValueError("Required model files are missing or absent from checksum records")
    for root in roots:
        for path in root.rglob("*"):
            if path.is_file() and path.suffix in (".safetensors", ".pt", ".bin", ".model", ".json"):
                if path.resolve() not in covered:
                    raise ValueError(f"Unrecorded model/config file: {path}")
    return manifest


def expected_packages(backend):
    result = {}
    for line in (HERE / backend / "requirements.lock.txt").read_text().splitlines():
        if line and not line.startswith("#"):
            name, version = line.split("==", 1)
            result[name] = version
    return result


def verify_existing(directory, backend, pin, require_complete=False):
    manifest = verify_manifest(directory, backend, pin)
    python = directory / ".venv/Scripts/python.exe"
    if not python.is_file():
        raise ValueError(f"Missing deployment Python: {python}")
    if run(["git", "-C", directory / "source", "rev-parse", "HEAD"]) != pin["source_commit"]:
        raise ValueError("Deployment source revision mismatch")
    if run(["git", "-C", directory / "source", "status", "--porcelain", "--untracked-files=no"]):
        raise ValueError("Deployment source has local modifications")
    probe = ("import sys,json,importlib.metadata as m; "
             "print(json.dumps({'python':'.'.join(map(str,sys.version_info[:3])),"
             "'packages':{d.metadata['Name'].lower().replace('_','-'):d.version "
             "for d in m.distributions()}}))")
    installed = json.loads(run([python, "-c", probe]))
    pins = read_json(HERE / "pins.json")
    if installed["python"] != pins["python"]:
        raise ValueError("Deployment Python version mismatch")
    for name, version in expected_packages(backend).items():
        if installed["packages"].get(name.lower().replace("_", "-")) != version:
            raise ValueError(f"Deployment dependency mismatch: {name}=={version}")
    for verb in ("start", "stop", "check"):
        if not (directory / f"{verb}-{backend}.ps1").is_file():
            raise ValueError(f"Missing {verb}-{backend}.ps1")
    if backend == "laya" and not (directory / "serve-local.py").is_file():
        raise ValueError("Missing Laya strict-budget service entrypoint")
    if require_complete and not (directory / "deployment-complete.json").is_file():
        raise ValueError("Incomplete installation; choose a new empty deployment root")
    return manifest


def installation_environment(directory, online=False):
    env = dict(os.environ, PYTHONUTF8="1", PYTHONUNBUFFERED="1",
               UV_PYTHON_INSTALL_DIR=str(directory / "python"),
               UV_PYTHON_BIN_DIR=str(directory / "tools"),
               UV_CACHE_DIR=str(directory / "cache/uv"),
               HF_HOME=str(directory / "models"), HF_HUB_CACHE=str(directory / "models/hub"),
               HF_HUB_DISABLE_SYMLINKS_WARNING="1", TORCH_HOME=str(directory / "cache/torch"),
               TEMP=str(directory / "tmp"), TMP=str(directory / "tmp"))
    if online:
        env.pop("HF_HUB_OFFLINE", None)
        env.pop("TRANSFORMERS_OFFLINE", None)
    return env


def install_new(directory, backend, pin, pins, project, uv):
    # Claim a new directory exclusively. Never adopt or repair a partial installation.
    directory.mkdir(parents=True, exist_ok=False)
    for name in ("logs", "tmp", "tools", "cache", "provenance"):
        (directory / name).mkdir()
    env = installation_environment(directory, online=True)
    for source in (HERE / backend).iterdir():
        if source.is_file():
            shutil.copy2(source, directory / source.name)
    shutil.copy2(HERE / "download_models.py", directory / "download_models.py")
    (directory / "pins.json").write_text(json.dumps(pin, indent=2), encoding="utf-8")
    (directory / "project.json").write_text(json.dumps({"project": str(project)}, indent=2), encoding="utf-8")
    run(["git", "clone", "--no-checkout", pin["source_url"], directory / "source"], env=env)
    run(["git", "-C", directory / "source", "checkout", "--detach", pin["source_commit"]], env=env)
    run([*uv, "python", "install", pins["python"]], env=env)
    interpreter = run([*uv, "python", "find", pins["python"], "--managed-python"], env=env)
    run([*uv, "venv", "--python", interpreter, directory / ".venv"], env=env)
    python = directory / ".venv/Scripts/python.exe"
    # Install CUDA torch from its own index, then the remaining pinned dependencies.
    run([*uv, "pip", "install", "--python", python, "--index-url", pins["torch_index"],
         "torch==2.8.0+cu128"], env=env)
    remaining = [f"{name}=={version}" for name, version in expected_packages(backend).items() if name != "torch"]
    run([*uv, "pip", "install", "--python", python, *remaining], env=env)
    run([*uv, "pip", "install", "--python", python, "--no-deps", "-e", directory / "source"], env=env)
    run([*uv, "pip", "check", "--python", python], env=env)
    run([python, "-c", "import torch; assert torch.cuda.is_available(), 'CUDA unavailable'; "
         "assert torch.cuda.is_bf16_supported(), 'BF16 unavailable'; print(torch.cuda.get_device_name(0))"], env=env)
    subprocess.run([str(python), str(directory / "download_models.py")], env=env, check=True)
    verify_existing(directory, backend, pin)
    (directory / "deployment-complete.json").write_text(
        json.dumps({"backend": backend, "source_commit": pin["source_commit"],
                    "revision": pin["revision"]}, indent=2), encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("laya", "kev", "both"), default="both")
    parser.add_argument("--deployment-root", type=Path, default=Path("D:/AI"))
    parser.add_argument("--project", type=Path, default=HERE.parents[1])
    parser.add_argument("--uv", help="Path to uv.exe; otherwise use python -m uv")
    parser.add_argument("--verify-only", action="store_true")
    parser.add_argument("--plan", action="store_true", help="Print pinned install targets without writing or downloading")
    args = parser.parse_args(argv)
    pins = read_json(HERE / "pins.json")
    project = args.project.resolve()
    if not (project / "jev_api/boat_client.py").is_file():
        parser.error("ProjectPath must contain jev_api/boat_client.py")
    targets = ("laya", "kev") if args.backend == "both" else (args.backend,)
    root = args.deployment_root.resolve()
    for backend in targets:
        pin = pins[backend]
        if not re.fullmatch(r"[0-9a-f]{40}", pin["revision"]):
            raise ValueError("Expected immutable model revision")
        print(json.dumps({**pin, "backend": backend, "directory": str(root / pin["directory"])}, ensure_ascii=False), flush=True)
    if args.plan:
        return 0
    if os.name != "nt":
        parser.error("Deployment requires Windows with an NVIDIA CUDA/BF16 GPU")
    # Preflight every existing target before installing anything else.
    existing = set()
    for backend in targets:
        directory = root / pins[backend]["directory"]
        if directory.exists():
            if (directory / "pins.json").exists() and not (directory / "deployment-complete.json").exists():
                raise ValueError(f"Incomplete installation retained at {directory}; choose a new root")
            verify_existing(directory, backend, pins[backend])
            existing.add(backend)
            print(f"Verified existing {backend}; reused without writing: {directory}", flush=True)
        elif args.verify_only:
            raise ValueError(f"Missing deployment: {directory}")
    if args.verify_only:
        return 0
    uv = [args.uv] if args.uv else [sys.executable, "-m", "uv"]
    if len(existing) != len(targets):
        run([*uv, "--version"])
        run(["git", "--version"])
    for backend in targets:
        if backend not in existing:
            directory = root / pins[backend]["directory"]
            print(f"Installing {backend} at {directory}; downloads may take several minutes...", flush=True)
            install_new(directory, backend, pins[backend], pins, project, uv)
            print(f"Installed {backend}: {directory}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        print(f"Deployment failed: {error}", file=sys.stderr)
        raise SystemExit(1)
