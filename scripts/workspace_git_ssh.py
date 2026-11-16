#!/usr/bin/env python3
"""Validate workspace imports and migrate Git remote URLs to SSH."""

import os
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import urlsplit

import yaml


def ssh_url(value):
    """Convert known GitHub clone URLs; require explicit SSH for other hosts."""
    if not isinstance(value, str) or not value or any(c.isspace() or c == "\0" for c in value):
        raise ValueError("Repository URLs must be nonempty and contain no whitespace.")
    parsed = urlsplit(value)
    if parsed.scheme in ("https", "http", "git"):
        if (parsed.hostname != "github.com" or parsed.username or parsed.password
                or parsed.port or parsed.query or parsed.fragment):
            raise ValueError("Only GitHub HTTP(S)/git URLs can be converted; use an explicit SSH URL.")
        path = parsed.path.strip("/")
        if not re.fullmatch(r"[\w.-]+/[\w.-]+", path) or ".." in path.split("/"):
            raise ValueError("GitHub URLs must identify an owner and repository.")
        return "git@github.com:" + path
    if parsed.scheme == "ssh":
        if (not parsed.hostname or not parsed.username or parsed.password
                or parsed.path in ("", "/") or parsed.query or parsed.fragment):
            raise ValueError("Invalid SSH repository URL.")
        if parsed.hostname == "github.com" and parsed.username == "git" and parsed.port in (None, 22):
            return "git@github.com:" + parsed.path.lstrip("/")
        return value
    if re.fullmatch(r"[^@\s/:]+@[^\s/:]+:[^\s]+", value):
        return value
    raise ValueError("Repository URLs must use SSH (git@host:path or ssh://user@host/path).")


def safe_path(base, relative):
    if (not isinstance(relative, str) or not relative or "\0" in relative
            or os.path.isabs(relative) or any(p in ("..", ".git") for p in relative.split("/"))):
        raise ValueError(f"Unsafe repository path: {relative!r}")
    result = (base / relative).resolve()
    if result == base or not result.is_relative_to(base):
        raise ValueError(f"Repository path escapes its import root: {relative!r}")
    return result


def load_manifest(manifest_path, base):
    with open(manifest_path, encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)
    repositories = manifest.get("repositories") if isinstance(manifest, dict) else None
    if not isinstance(repositories, dict):
        raise ValueError("Manifest must contain a 'repositories' mapping.")
    for path, specification in repositories.items():
        safe_path(base, path)
        if (not isinstance(specification, dict) or specification.get("type") != "git"
                or not isinstance(specification.get("version"), str)
                or not specification["version"] or "\0" in specification["version"]):
            raise ValueError(f"Repository {path!r} must define git type, url, and version.")
        try:
            specification["url"] = ssh_url(specification.get("url"))
        except ValueError as error:
            raise ValueError(f"Repository {path!r}: {error}") from error
    return repositories


def git(repo, *args, absent_ok=False):
    result = subprocess.run(["git", "-C", str(repo), *args], check=False,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if result.returncode and not (absent_ok and result.returncode == 1):
        raise RuntimeError(result.stderr.strip() or f"Git command failed in {repo}")
    return result.stdout


def is_repository(repo):
    if not (repo / ".git").exists():
        return False
    return Path(git(repo, "rev-parse", "--show-toplevel").strip()).resolve() == repo


def config_entries(repo, pattern, config_file=None):
    source = ["--file", str(config_file)] if config_file else ["--local"]
    output = git(repo, "config", *source, "--null", "--get-regexp", pattern, absent_ok=True)
    entries = {}
    for record in output.split("\0"):
        if record:
            key, value = record.split("\n", 1)
            entries.setdefault(key, []).append(value)
    return entries


def migrate_repositories(base, repositories):
    pending = [base, *(safe_path(base, path) for path in repositories)]
    seen = set()
    changes = []
    while pending:
        repo = pending.pop()
        if repo in seen or not is_repository(repo):
            continue
        seen.add(repo)
        settings = config_entries(repo, r"^remote\..*\.(url|pushurl)$")
        modules_file = repo / ".gitmodules"
        if modules_file.is_file():
            modules = config_entries(repo, r"^submodule\..*\.(url|path)$", modules_file)
            for key, values in modules.items():
                if key.endswith(".path"):
                    pending.append(safe_path(repo, values[-1]))
                else:
                    settings[key] = config_entries(repo, "^" + re.escape(key) + "$").get(key, values)
        for key, values in settings.items():
            # Relative submodule URLs inherit their parent's SSH transport.
            updated = [value if key.startswith("submodule.") and value.startswith(("./", "../"))
                       else ssh_url(value) for value in values]
            if updated != values:
                changes.append((repo, key, updated))
    # Validate every URL before modifying any repository configuration.
    for repo, key, values in changes:
        git(repo, "config", "--local", "--replace-all", key, values[0])
        for value in values[1:]:
            git(repo, "config", "--local", "--add", key, value)
        print(f"[ssh:updated] {repo.relative_to(base) or '.'}: {key}")
    print(f"SSH remote check: repositories={len(seen)} updated_settings={len(changes)}")


def main():
    mode, manifest_path, import_path, *options = sys.argv[1:]
    base = Path(import_path).resolve()
    repositories = load_manifest(manifest_path, base)
    migrate_repositories(base, repositories)
    if mode == "prepare":
        output_dir, force = options
        output_dir = Path(output_dir)
        imports = {path: spec for path, spec in repositories.items()
                   if force == "1" or not is_repository(safe_path(base, path))}
        if imports:
            (output_dir / "import.repos").write_text(
                yaml.safe_dump({"repositories": imports}, sort_keys=False), encoding="utf-8")
        with (output_dir / "repositories.bin").open("wb") as stream:
            for path, spec in repositories.items():
                for value in (path, spec["url"], spec["version"]):
                    stream.write(value.encode() + b"\0")
    elif mode != "migrate":
        raise ValueError(f"Unknown mode: {mode}")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, RuntimeError, OSError, yaml.YAMLError) as error:
        raise SystemExit(f"SSH import preflight failed: {error}") from error
