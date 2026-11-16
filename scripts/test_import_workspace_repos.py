#!/usr/bin/env python3
"""Exercise SSH imports and safe synchronization with real local Git fixtures."""

import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
IMPORTER = WORKSPACE / "import_workspace_repos.sh"


class WorkspaceSshImportTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="ats_git_ssh_", dir="/tmp")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.workspace = self.root / "workspace with spaces"
        self.workspace.mkdir()
        self.remote_root = self.root / "remotes"
        self.remote = self.remote_root / "team/source.git"
        self.remote.parent.mkdir(parents=True)
        self.seed = self.root / "seed"
        self.log = self.root / "ssh.log"
        self.env = {key: value for key, value in os.environ.items()
                    if not key.startswith("GIT_") and key not in ("MANIFEST", "IMPORT_PATH")}
        self.env.update({
            "GIT_CONFIG_NOSYSTEM": "1",
            "GIT_CONFIG_GLOBAL": "/dev/null",
            "GIT_TERMINAL_PROMPT": "0",
            "GIT_AUTHOR_NAME": "ATS import test",
            "GIT_AUTHOR_EMAIL": "ats-test@example.invalid",
            "GIT_COMMITTER_NAME": "ATS import test",
            "GIT_COMMITTER_EMAIL": "ats-test@example.invalid",
            "GIT_SSH_VARIANT": "ssh",
            "FIXTURE_REMOTE_ROOT": str(self.remote_root),
            "FIXTURE_SSH_LOG": str(self.log),
            "IMPORT_PATH": str(self.workspace),
            "VCS_WORKERS": "1",
            "NETWORK_ATTEMPTS": "3",
        })
        ssh = self.root / "fixture_ssh.py"
        ssh.write_text("""#!/usr/bin/env python3
import os
from pathlib import Path
import shlex
import subprocess
import sys

command = shlex.split(sys.argv[-1])
if len(command) != 2 or command[0] != 'git-upload-pack':
    raise SystemExit('Unexpected SSH fixture command')
root = Path(os.environ['FIXTURE_REMOTE_ROOT']).resolve()
repo = (root / command[1].lstrip('/')).resolve()
if not repo.is_relative_to(root):
    raise SystemExit('SSH fixture path escapes remote root')
with open(os.environ['FIXTURE_SSH_LOG'], 'a') as stream:
    stream.write(command[1] + '\\n')
fail_file = os.environ.get('FIXTURE_SSH_FAIL_FILE')
if fail_file:
    fail_file = Path(fail_file)
    remaining = int(fail_file.read_text())
    if remaining:
        fail_file.write_text(str(remaining - 1))
        raise SystemExit('simulated SSH connection closed')
raise SystemExit(subprocess.run(['git-upload-pack', str(repo)]).returncode)
""", encoding="utf-8")
        ssh.chmod(0o755)
        self.env["GIT_SSH_COMMAND"] = shlex.quote(str(ssh))
        self.git("init", "--bare", "--initial-branch=main", str(self.remote))
        self.git("init", "--initial-branch=main", str(self.seed))
        (self.seed / "tracked.txt").write_text("first\n", encoding="utf-8")
        self.git("-C", str(self.seed), "add", "tracked.txt")
        self.git("-C", str(self.seed), "commit", "-m", "fixture baseline")
        self.git("-C", str(self.seed), "push", str(self.remote), "main")
        self.initial_head = self.git("-C", str(self.seed), "rev-parse", "HEAD")
        self.manifest = self.root / "custom.repos"
        self.write_manifest()

    def git(self, *args):
        result = subprocess.run(["git", *args], env=self.env, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result.stdout.strip()

    def write_manifest(self, url="https://github.com/team/source.git", version="main", repositories=None):
        if repositories is None:
            repositories = {"src/source": {"type": "git", "url": url, "version": version}}
        self.manifest.write_text(yaml.safe_dump({"repositories": repositories}), encoding="utf-8")

    def run_import(self, *args, expected=0):
        result = subprocess.run(["bash", str(IMPORTER), "--manifest", str(self.manifest), *args],
                                env=self.env, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, timeout=45)
        output = result.stdout + result.stderr
        self.assertEqual(result.returncode, expected, output)
        return output

    def existing(self, url="https://github.com/team/source.git"):
        repo = self.workspace / "src/source"
        self.git("clone", str(self.remote), str(repo))
        self.git("-C", str(repo), "remote", "set-url", "origin", url)
        return repo

    def advance_remote(self):
        (self.seed / "tracked.txt").write_text("second\n", encoding="utf-8")
        self.git("-C", str(self.seed), "commit", "-am", "fixture update")
        self.git("-C", str(self.seed), "push", str(self.remote), "main")
        return self.git("-C", str(self.seed), "rev-parse", "HEAD")

    def test_both_workspace_manifests_use_ssh(self):
        for name in ("dependencies.repos", "dependencies.lock.repos"):
            with self.subTest(manifest=name):
                manifest = yaml.safe_load((WORKSPACE / name).read_text(encoding="utf-8"))
                for spec in manifest["repositories"].values():
                    self.assertTrue(spec["url"].startswith(("git@", "ssh://")), spec["url"])

    def test_https_manifest_clones_over_ssh_without_editing_manifest(self):
        manifest_before = self.manifest.read_bytes()
        self.run_import("--no-sync")
        repo = self.workspace / "src/source"
        self.assertEqual(self.git("-C", str(repo), "remote", "get-url", "origin"),
                         "git@github.com:team/source.git")
        self.assertEqual(self.git("-C", str(repo), "rev-parse", "HEAD"), self.initial_head)
        self.assertIn("team/source.git", self.log.read_text())
        self.assertEqual(self.manifest.read_bytes(), manifest_before)

    def test_dirty_repo_migrates_all_urls_without_fetch_or_file_changes(self):
        repo = self.existing()
        self.git("-C", str(repo), "config", "--add", "remote.origin.url",
                 "http://github.com/team/mirror.git")
        self.git("-C", str(repo), "config", "--add", "remote.origin.pushurl",
                 "git://github.com/team/source.git")
        self.git("-C", str(repo), "remote", "add", "upstream", "https://github.com/other/source.git")
        (repo / "tracked.txt").write_text("user edit\n", encoding="utf-8")
        (repo / "untracked.txt").write_text("user file\n", encoding="utf-8")
        status_before = self.git("-C", str(repo), "status", "--porcelain")
        self.run_import("--no-sync")
        self.assertEqual(self.git("-C", str(repo), "config", "--get-all", "remote.origin.url"),
                         "git@github.com:team/source.git\ngit@github.com:team/mirror.git")
        self.assertEqual(self.git("-C", str(repo), "config", "--get", "remote.origin.pushurl"),
                         "git@github.com:team/source.git")
        self.assertEqual(self.git("-C", str(repo), "remote", "get-url", "upstream"),
                         "git@github.com:other/source.git")
        output = self.run_import(expected=1)
        self.assertIn("working tree is not clean", output)
        self.assertFalse(self.log.exists())
        self.assertEqual((repo / "tracked.txt").read_text(), "user edit\n")
        self.assertEqual((repo / "untracked.txt").read_text(), "user file\n")
        self.assertEqual(self.git("-C", str(repo), "status", "--porcelain"), status_before)
        self.assertEqual(self.git("-C", str(repo), "rev-parse", "HEAD"), self.initial_head)

    def test_clean_existing_repo_fast_forwards_over_ssh(self):
        repo = self.existing()
        updated_head = self.advance_remote()
        output = self.run_import()
        self.assertIn("[sync:updated]", output)
        self.assertEqual(self.git("-C", str(repo), "rev-parse", "HEAD"), updated_head)
        self.assertEqual((repo / "tracked.txt").read_text(), "second\n")
        self.assertIn("team/source.git", self.log.read_text())

    def test_no_sync_does_not_even_fetch_clean_existing_repo(self):
        repo = self.existing()
        self.advance_remote()
        self.run_import("--no-sync")
        self.assertFalse(self.log.exists())
        self.assertEqual(self.git("-C", str(repo), "rev-parse", "origin/main"), self.initial_head)

    def test_transient_ssh_failures_retry_and_finish_fast_forward(self):
        repo = self.existing()
        updated_head = self.advance_remote()
        failure_count = self.root / "ssh_failures.txt"
        failure_count.write_text("2", encoding="utf-8")
        self.env["FIXTURE_SSH_FAIL_FILE"] = str(failure_count)
        self.env["NETWORK_ATTEMPTS"] = "3"
        output = self.run_import()
        self.assertEqual(output.count("[network:retry]"), 2)
        self.assertEqual(failure_count.read_text(), "0")
        self.assertEqual(self.git("-C", str(repo), "rev-parse", "HEAD"), updated_head)

    def test_persistent_ssh_failure_stops_without_advancing_head(self):
        repo = self.existing()
        self.advance_remote()
        failure_count = self.root / "ssh_failures.txt"
        failure_count.write_text("5", encoding="utf-8")
        self.env["FIXTURE_SSH_FAIL_FILE"] = str(failure_count)
        self.env["NETWORK_ATTEMPTS"] = "2"
        output = self.run_import(expected=1)
        self.assertEqual(output.count("[network:retry]"), 1)
        self.assertEqual(failure_count.read_text(), "3")
        self.assertEqual(self.git("-C", str(repo), "rev-parse", "HEAD"), self.initial_head)
        self.assertEqual(self.git("-C", str(repo), "status", "--porcelain"), "")

    def test_different_origin_keeps_repository_identity_and_skips_sync(self):
        repo = self.existing(url="https://github.com/other/source.git")
        output = self.run_import(expected=1)
        self.assertIn("origin URL differs", output)
        self.assertEqual(self.git("-C", str(repo), "remote", "get-url", "origin"),
                         "git@github.com:other/source.git")
        self.assertEqual(self.git("-C", str(repo), "rev-parse", "HEAD"), self.initial_head)
        self.assertFalse(self.log.exists())

    def test_pinned_shallow_import_keeps_requested_commit(self):
        self.advance_remote()
        self.write_manifest(version=self.initial_head)
        self.run_import("--shallow", "--no-sync")
        repo = self.workspace / "src/source"
        self.assertEqual(self.git("-C", str(repo), "rev-parse", "HEAD"), self.initial_head)
        self.assertEqual(self.git("-C", str(repo), "branch", "--show-current"), "")

    def test_invalid_later_entry_is_rejected_before_any_config_or_network_change(self):
        repo = self.existing()
        for path in ("../outside", "/tmp/outside", ".git/objects", "."):
            with self.subTest(path=path):
                self.write_manifest(repositories={
                    "src/source": {"type": "git", "url": "https://github.com/team/source.git", "version": "main"},
                    path: {"type": "git", "url": "git@github.com:team/source.git", "version": "main"},
                })
                self.run_import("--force", expected=1)
                self.assertEqual(self.git("-C", str(repo), "config", "--get", "remote.origin.url"),
                                 "https://github.com/team/source.git")
                self.assertFalse(self.log.exists())

    def test_symlink_cannot_import_outside_workspace(self):
        outside = self.root / "outside"
        outside.mkdir()
        (self.workspace / "src").symlink_to(outside, target_is_directory=True)
        self.run_import("--force", expected=1)
        self.assertEqual(list(outside.iterdir()), [])
        self.assertFalse(self.log.exists())

    def test_unknown_https_host_requires_explicit_ssh_url(self):
        self.write_manifest(url="https://git.example.invalid/team/source.git")
        self.run_import("--no-sync", expected=1)
        self.assertFalse((self.workspace / "src").exists())
        self.assertFalse(self.log.exists())

    def add_https_submodule(self):
        module_remote = self.remote_root / "team/module.git"
        self.git("clone", "--bare", str(self.remote), str(module_remote))
        self.git("-C", str(self.seed), "-c", "protocol.file.allow=always", "submodule", "add",
                 str(module_remote), "module")
        self.git("-C", str(self.seed), "config", "--file", ".gitmodules",
                 "submodule.module.url", "https://github.com/team/module.git")
        self.git("-C", str(self.seed), "add", ".gitmodules", "module")
        self.git("-C", str(self.seed), "commit", "-m", "fixture HTTPS submodule")
        self.git("-C", str(self.seed), "push", str(self.remote), "main")

    def assert_ssh_submodule(self, repo):
        module = repo / "module"
        self.assertIn("team/module.git", self.log.read_text())
        self.assertEqual(self.git("-C", str(module), "config", "--get", "remote.origin.url"),
                         "git@github.com:team/module.git")
        self.assertEqual(self.git("-C", str(repo), "config", "--get", "submodule.module.url"),
                         "git@github.com:team/module.git")
        self.assertEqual(self.git("-C", str(repo), "status", "--porcelain"), "")

    def test_https_submodule_clones_over_ssh_and_persists_ssh_origins(self):
        self.add_https_submodule()
        self.run_import("--no-sync")
        repo = self.workspace / "src/source"
        self.assert_ssh_submodule(repo)

    def test_clean_existing_repo_initializes_submodule_over_ssh(self):
        self.add_https_submodule()
        repo = self.existing()
        self.assertFalse((repo / "module/.git").exists())
        self.run_import()
        self.assert_ssh_submodule(repo)

    def test_missing_submodule_reports_failure_and_keeps_working_files(self):
        self.add_https_submodule()
        self.git("-C", str(self.seed), "config", "--file", ".gitmodules",
                 "submodule.module.url", "git@github.com:team/missing.git")
        self.git("-C", str(self.seed), "commit", "-am", "fixture missing submodule")
        self.git("-C", str(self.seed), "push", str(self.remote), "main")
        repo = self.existing()
        self.env["NETWORK_ATTEMPTS"] = "1"
        output = self.run_import(expected=1)
        self.assertIn("could not synchronize submodules", output)
        self.assertIn("failed=1", output)
        self.assertEqual((repo / "tracked.txt").read_text(), "first\n")
        self.assertEqual(self.git("-C", str(repo), "status", "--porcelain"), "")

    def test_missing_manifest_argument_has_clear_error(self):
        result = subprocess.run(["bash", str(IMPORTER), "--manifest"], env=self.env,
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
        self.assertEqual(result.returncode, 2)
        self.assertIn("requires a file path", result.stderr)


if __name__ == "__main__":
    unittest.main()
