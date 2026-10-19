"""Deployment safety tests with tiny fixtures; no downloads or real processes are stopped."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent / "LocalModels"
spec = importlib.util.spec_from_file_location("local_model_deployment", HERE / "setup.py")
deployment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(deployment)
PINS = deployment.read_json(HERE / "pins.json")


class DeploymentTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="model deployment with spaces ")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def fixture(self, backend):
        directory = self.root / PINS[backend]["directory"]
        directory.mkdir()
        model = directory / "models/snapshot"
        model.mkdir(parents=True)
        pin = PINS[backend]
        manifest = {"source_commit": pin["source_commit"], "files": {}}
        if backend == "laya":
            manifest.update(model_id=pin["model_id"], revision=pin["revision"], path=str(model))
            files = ["model.safetensors", "rl_agent_config.json", "encoder/config.json",
                     "tokenizer/tokenizer.json", "tokenizer/tokenizer_config.json"]
        else:
            base = directory / "models/base"
            base.mkdir()
            manifest.update(kev_id=pin["model_id"], kev_revision=pin["revision"], kev_path=str(model),
                            base_id=pin["base_id"], base_revision=pin["base_revision"], base_path=str(base))
            files = ["head.pt", "adapter_config.json", "adapter_model.safetensors"]
            for name in ("config.json", "model.safetensors"):
                path = base / name
                path.write_bytes(b"base")
                manifest["files"][str(path)] = {"bytes": 4, "sha256": deployment.file_hash(path)}
        for name in files:
            path = model / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"{}")
            manifest["files"][str(path) if backend == "kev" else name] = {
                "bytes": 2, "sha256": deployment.file_hash(path)}
        (directory / "models.json").write_text(json.dumps(manifest), encoding="utf-8")
        return directory, model, manifest

    def test_manifest_supports_spaces_and_both_backends(self):
        for backend in ("laya", "kev"):
            directory, _, manifest = self.fixture(backend)
            self.assertEqual(deployment.verify_manifest(directory, backend, PINS[backend]), manifest)

    def test_rejects_wrong_revision(self):
        directory, _, manifest = self.fixture("laya")
        manifest["revision"] = "latest"
        (directory / "models.json").write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, "revision mismatch"):
            deployment.verify_manifest(directory, "laya", PINS["laya"])

    def test_rejects_missing_and_changed_weights(self):
        directory, model, _ = self.fixture("laya")
        weights = model / "model.safetensors"
        weights.write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "checksum mismatch"):
            deployment.verify_manifest(directory, "laya", PINS["laya"])
        weights.unlink()
        with self.assertRaises(FileNotFoundError):
            deployment.verify_manifest(directory, "laya", PINS["laya"])

    def test_rejects_checksum_records_outside_deployment(self):
        directory, _, manifest = self.fixture("laya")
        manifest["files"]["../../../../outside"] = {"bytes": 0, "sha256": "0" * 64}
        (directory / "models.json").write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, "outside"):
            deployment.verify_manifest(directory, "laya", PINS["laya"])

    def test_rejects_missing_checksum_coverage(self):
        directory, _, manifest = self.fixture("kev")
        manifest["files"] = {name: value for name, value in manifest["files"].items() if "head.pt" not in name}
        (directory / "models.json").write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, "Required model files"):
            deployment.verify_manifest(directory, "kev", PINS["kev"])

    def test_allows_only_official_laya_tokenizer_normalization(self):
        directory, model, manifest = self.fixture("laya")
        path = model / "tokenizer/tokenizer_config.json"
        original = {"tokenizer_class": "TokenizersBackend", "backend": "tokenizers",
                    "extra_special_tokens": ["[action]"]}
        path.write_text(json.dumps(original))
        manifest["files"]["tokenizer/tokenizer_config.json"] = {
            "bytes": path.stat().st_size, "sha256": deployment.file_hash(path)}
        provenance = directory / "provenance"
        provenance.mkdir()
        (provenance / "original-tokenizer-config.json").write_bytes(path.read_bytes())
        (directory / "models.json").write_text(json.dumps(manifest))
        normalized = deployment.normalized_tokenizer(original)
        path.write_text(json.dumps(normalized))
        deployment.verify_manifest(directory, "laya", PINS["laya"])
        path.write_text(json.dumps({**normalized, "other_change": True}))
        with self.assertRaisesRegex(ValueError, "checksum mismatch"):
            deployment.verify_manifest(directory, "laya", PINS["laya"])

    def test_install_never_overwrites_existing_directory(self):
        target = self.root / "existing"
        target.mkdir()
        marker = target / "keep"
        marker.write_text("original")
        with patch.object(deployment, "run") as run:
            with self.assertRaises(FileExistsError):
                deployment.install_new(target, "laya", PINS["laya"], PINS, self.root, ["uv"])
        run.assert_not_called()
        self.assertEqual(marker.read_text(), "original")

    def test_plan_is_read_only_and_shows_fixed_revisions(self):
        missing = self.root / "not created"
        output = io.StringIO()
        with contextlib.redirect_stdout(output), patch.object(deployment, "run") as run:
            self.assertEqual(deployment.main(["--deployment-root", str(missing), "--plan"]), 0)
        self.assertFalse(missing.exists())
        run.assert_not_called()
        self.assertIn(PINS["kev"]["revision"], output.getvalue())
        self.assertIn(PINS["laya"]["revision"], output.getvalue())

    def test_verify_only_missing_deployment_never_installs(self):
        with contextlib.redirect_stdout(io.StringIO()), patch.object(deployment, "install_new") as install:
            with self.assertRaisesRegex(ValueError, "Missing deployment"):
                deployment.main(["--deployment-root", str(self.root), "--verify-only"])
        install.assert_not_called()

    def test_partial_install_is_retained_and_never_repaired(self):
        target = self.root / "Laya"
        target.mkdir()
        (target / "pins.json").write_text("{}")
        with contextlib.redirect_stdout(io.StringIO()), patch.object(deployment, "install_new") as install:
            with self.assertRaisesRegex(ValueError, "Incomplete installation"):
                deployment.main(["--backend", "laya", "--deployment-root", str(self.root)])
        install.assert_not_called()
        self.assertTrue((target / "pins.json").exists())

    def test_online_install_does_not_inherit_offline_flags(self):
        with patch.dict(deployment.os.environ, HF_HUB_OFFLINE="1", TRANSFORMERS_OFFLINE="1"):
            env = deployment.installation_environment(self.root, online=True)
        self.assertNotIn("HF_HUB_OFFLINE", env)
        self.assertNotIn("TRANSFORMERS_OFFLINE", env)

    def test_download_uses_immutable_revisions_and_records_hashes(self):
        from unittest.mock import Mock
        pin = PINS["kev"]
        (self.root / "pins.json").write_text(json.dumps(pin))
        model = self.root / "model"
        base = self.root / "base"
        model.mkdir()
        base.mkdir()
        (model / "adapter_config.json").write_text(json.dumps({"base_model_name_or_path": pin["base_id"]}))
        (model / "head.pt").write_bytes(b"test head")
        (base / "model.safetensors").write_bytes(b"test base")
        snapshot = Mock(side_effect=[str(model), str(base)])
        torch = types.SimpleNamespace(load=Mock(return_value={"base": pin["base_id"], "base_revision": pin["base_revision"]}))
        modules = {"huggingface_hub": types.SimpleNamespace(snapshot_download=snapshot), "torch": torch}
        with patch.dict(sys.modules, modules):
            download_spec = importlib.util.spec_from_file_location("pinned_download", HERE / "download_models.py")
            download = importlib.util.module_from_spec(download_spec)
            download_spec.loader.exec_module(download)
            with contextlib.redirect_stdout(io.StringIO()):
                download.download(self.root)
            self.assertEqual(snapshot.call_args_list[0].kwargs["revision"], pin["revision"])
            self.assertEqual(snapshot.call_args_list[1].kwargs["revision"], pin["base_revision"])
            self.assertTrue(torch.load.call_args.kwargs["weights_only"])
            manifest = deployment.read_json(self.root / "models.json")
            self.assertEqual(manifest["files"][str(model / "head.pt")]["sha256"], deployment.file_hash(model / "head.pt"))
            with self.assertRaises(FileExistsError):
                download.download(self.root)
            self.assertEqual(snapshot.call_count, 2)

    def test_existing_deployments_are_verified_before_any_install(self):
        (self.root / "Kev-0.8B").mkdir()
        with contextlib.redirect_stdout(io.StringIO()), patch.object(deployment, "verify_existing", side_effect=ValueError("wrong version")):
            with patch.object(deployment, "install_new") as install:
                with self.assertRaisesRegex(ValueError, "wrong version"):
                    deployment.main(["--deployment-root", str(self.root)])
                install.assert_not_called()

    def test_powershell_wrapper_preserves_space_paths(self):
        result = subprocess.run(["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
                                 "-File", str(HERE / "setup.ps1"), "-Backend", "kev",
                                 "-DeploymentRoot", str(self.root / "custom root"), "-Plan"],
                                capture_output=True, text=True, encoding="utf-8")
        self.assertEqual(result.returncode, 0, result.stderr)
        record = json.loads(result.stdout)
        self.assertEqual(Path(record["directory"]), self.root / "custom root/Kev-0.8B")
        self.assertFalse((self.root / "custom root").exists())


if __name__ == "__main__":
    unittest.main()
