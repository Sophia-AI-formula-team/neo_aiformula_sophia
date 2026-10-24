#!/usr/bin/env python3
"""Offline, reviewed planner/lab-agent handoffs; Python standard library only.

This tool never runs Git, downloads artifacts, contacts a service or operates
hardware. Automated checks are conservative tripwires, not proof of redaction.
The author must manually review every published file, including images.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys
import tempfile
from urllib.parse import urlsplit
import zipfile


MAX_FILE_BYTES = 5 * 1024 * 1024
MAX_TOTAL_BYTES = 20 * 1024 * 1024
MAX_FILES = 512  # Including the generated hashmanifest.json.
EXTENSIONS = {".txt", ".log", ".json", ".jsonl", ".yaml", ".yml", ".csv",
              ".tsv", ".md", ".xml", ".png", ".jpg", ".patch", ".diff"}
BINARY_EXTENSIONS = {".png", ".jpg"}
SECTIONS = ("Objective and scope", "Input handoff", "Source and working tree",
            "Work performed", "Evidence and result", "Risks and blockers", "Next agent actions")
SAFE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}\Z")
HEX40 = re.compile(r"[0-9a-fA-F]{40}\Z")
HEX64 = re.compile(r"[0-9a-fA-F]{64}\Z")
PLACEHOLDER = re.compile(r"<<[^>]*>>|\b(?:TODO|TBD|FIXME)\b", re.I)
SECRET_PATTERNS = (
    re.compile(r"-----BEGIN (?:[A-Z0-9 ]* )?PRIVATE KEY-----"),
    re.compile(r"\b(?:gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,})\b"),
    re.compile(r"\b(?:AKIA|ASIA)[A-Z0-9]{16}\b"),
    re.compile(r"\b(?:sk-(?:proj-|svcacct-)?[A-Za-z0-9_-]{20,}|xox[baprs]-[A-Za-z0-9-]{16,})\b"),
    re.compile(r"\bBearer\s+[A-Za-z0-9._~+/-]{12,}", re.I),
    re.compile(r"\beyJ[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{8,}\b"),
    re.compile(r"[a-z][a-z0-9+.-]*://[^\s/@:]+:[^\s/@]+@", re.I),
)
SECRET_ASSIGNMENT = re.compile(
    r'''\b(?:password|passwd|client_secret|private_key|secret|api[_-]?key|access[_-]?token|refresh[_-]?token|token)\b["']?\s*[:=]\s*["']?([^\s"',;}]+)''', re.I)
REDACTED = {"redacted", "[redacted]", "<redacted>", "none", "null", "not_set", "<not-stored>"}
GPS_PATTERN = re.compile(
    r'''(?:\b(?:latitude(?:_deg)?|longitude(?:_deg)?|lat|lon)\b["']?\s*[:=]\s*["']?[-+]?\d+(?:\.\d+)?|\b(?:origin_lla|poslla|lla|gps_coordinates)\b["']?\s*[:=]\s*\[\s*[-+]?\d)''', re.I)
GPS_TABLE = re.compile(
    r'''^[^\n]*\b(?:latitude(?:_deg)?|longitude(?:_deg)?|lat|lon)\b[^\n]*[,;\t][^\n]*\n[^\n]*[-+]?\d+\.\d+''', re.I | re.M)
REVIEW_WARNING = "Manual review remains required; scans cannot guarantee absence of secrets or private information, especially in images."


class HandoffError(ValueError):
    """A validation or exclusive-publication check failed."""


def _json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")


def _safe_id(value, label):
    if (not isinstance(value, str) or not SAFE_ID.fullmatch(value) or ".." in value
            or value.endswith(".") or value.split(".")[0].upper() in
            {"CON", "PRN", "AUX", "NUL", *("COM" + str(i) for i in range(1, 10)),
             *("LPT" + str(i) for i in range(1, 10))}):
        raise HandoffError("invalid " + label)
    return value


def _check_plain(path, allow_missing=False):
    """Reject symlinks/junctions, including ancestors, without following them."""
    path = Path(os.path.abspath(str(path)))
    for part in reversed((path,) + tuple(path.parents)):
        try:
            info = part.lstat()
        except FileNotFoundError:
            if allow_missing:
                continue
            raise HandoffError("path does not exist: " + str(part))
        if stat.S_ISLNK(info.st_mode) or getattr(info, "st_file_attributes", 0) & 0x400:
            raise HandoffError("symlinks and junctions are forbidden: " + str(part))
    return path


def _hidden(path, info):
    return path.name.startswith(".") or bool(getattr(info, "st_file_attributes", 0) & 0x2)


def _identity(info):
    return info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns


def _read_file(path, root):
    _check_plain(path)
    try:
        path.resolve(strict=True).relative_to(root)
    except ValueError:
        raise HandoffError("file escapes session directory")
    before = path.lstat()
    if not stat.S_ISREG(before.st_mode) or before.st_size > MAX_FILE_BYTES:
        raise HandoffError("nonregular or oversized file: " + path.name)
    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0) | getattr(os, "O_NOFOLLOW", 0)
    fd = os.open(str(path), flags)
    with os.fdopen(fd, "rb") as stream:
        opened = os.fstat(stream.fileno())
        if _identity(opened) != _identity(before):
            raise HandoffError("file changed while reading: " + path.name)
        data = stream.read(MAX_FILE_BYTES + 1)
        after = os.fstat(stream.fileno())
    _check_plain(path)
    if (len(data) > MAX_FILE_BYTES or _identity(after) != _identity(before)
            or _identity(path.lstat()) != _identity(before)):
        raise HandoffError("file changed or grew while reading: " + path.name)
    return data, _identity(before)


def _inventory(root):
    files = []
    nodes = 0
    pending = [root]
    while pending:
        directory = pending.pop()
        _check_plain(directory)
        for path in sorted(directory.iterdir(), key=lambda item: item.name):
            nodes += 1
            if nodes > MAX_FILES * 2:
                raise HandoffError("too many directory entries")
            info = path.lstat()
            _check_plain(path)
            relative = path.relative_to(root).as_posix()
            if _hidden(path, info):
                raise HandoffError("hidden files/directories are forbidden: " + relative)
            if any(char in path.name for char in ("\\", ":", "\x00", "\n", "\r")):
                raise HandoffError("unsafe filename")
            if path.is_dir():
                if relative != "evidence" and not relative.startswith("evidence/"):
                    raise HandoffError("only the evidence directory is allowed")
                pending.append(path)
                continue
            if not stat.S_ISREG(info.st_mode):
                raise HandoffError("only ordinary files are allowed")
            if relative not in ("session.json", "SESSION.md"):
                if not relative.startswith("evidence/") or path.suffix.lower() not in EXTENSIONS:
                    raise HandoffError("unapproved file: " + relative)
            if (path.name.lower().startswith(("id_rsa", "id_ed25519", "id_ecdsa"))
                    or path.suffix.lower() in {".pem", ".key", ".db3", ".mcap", ".bag"}):
                raise HandoffError("secret material or raw bag file is forbidden")
            files.append(path)
            if len(files) + 1 > MAX_FILES:
                raise HandoffError("too many files")
    return sorted(files, key=lambda item: item.relative_to(root).as_posix())


def _decode_json(data):
    def pairs(values):
        result = {}
        for key, value in values:
            if key in result:
                raise HandoffError("duplicate JSON field: " + key)
            result[key] = value
        return result

    def invalid(value):
        raise HandoffError("nonfinite JSON value: " + value)

    try:
        result = json.loads(data.decode("utf-8"), object_pairs_hook=pairs, parse_constant=invalid)
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise HandoffError("invalid session JSON: " + str(exc))
    if not isinstance(result, dict):
        raise HandoffError("session metadata must be an object")
    return result


def _metadata(meta, directory_name):
    fields = {"schema_version", "id", "role", "base_code_sha", "input_handoff_id",
              "consumed_handoff_id", "created_utc", "status", "result", "publication_review",
              "external_artifacts"}
    if not fields.issubset(meta) or set(meta) - fields - {"dependency_sources"}:
        raise HandoffError("missing or unknown session metadata fields")
    if type(meta["schema_version"]) is not int or meta["schema_version"] != 1:
        raise HandoffError("unsupported schema_version")
    if _safe_id(meta["id"], "session id") != directory_name:
        raise HandoffError("session id must match its directory name")
    _safe_id(meta["input_handoff_id"], "input handoff id")
    if meta["consumed_handoff_id"] != meta["input_handoff_id"]:
        raise HandoffError("consumed_handoff_id does not acknowledge the input handoff")
    if meta["role"] not in ("planner", "experiment") or meta["status"] not in ("draft", "handed_off"):
        raise HandoffError("invalid role or status")
    if meta["result"] not in ("pass", "partial", "blocked", "not_run"):
        raise HandoffError("invalid result")
    if not isinstance(meta["base_code_sha"], str) or not HEX40.fullmatch(meta["base_code_sha"]):
        raise HandoffError("base_code_sha must contain 40 hexadecimal characters")
    try:
        created = datetime.strptime(meta["created_utc"], "%Y-%m-%dT%H:%M:%SZ")
    except (ValueError, TypeError):
        raise HandoffError("created_utc must be a UTC timestamp ending in Z")
    if created.year < 1970:
        raise HandoffError("created_utc is out of range")
    review = meta["publication_review"]
    if (not isinstance(review, dict) or set(review) != {"secrets_checked", "privacy_checked"}
            or any(type(value) is not bool for value in review.values())):
        raise HandoffError("publication_review requires two explicit booleans")
    artifacts = meta["external_artifacts"]
    if not isinstance(artifacts, list) or len(artifacts) > 128:
        raise HandoffError("external_artifacts must be a bounded list")
    for artifact in artifacts:
        if not isinstance(artifact, dict) or set(artifact) != {"url", "sha256", "bytes", "access_note"}:
            raise HandoffError("external artifact needs url, sha256, bytes, access_note")
        if not isinstance(artifact["url"], str):
            raise HandoffError("external artifact URL must be text")
        url = urlsplit(artifact["url"])
        if (url.scheme != "https" or not url.hostname or url.username or url.password
                or url.query or url.fragment):
            raise HandoffError("external artifacts require stable credential-free HTTPS URLs without query or fragment")
        if not isinstance(artifact["sha256"], str) or not HEX64.fullmatch(artifact["sha256"]):
            raise HandoffError("external artifact SHA256 is required")
        if type(artifact["bytes"]) is not int or not 0 <= artifact["bytes"] <= 2 ** 63 - 1:
            raise HandoffError("external artifact byte length is invalid")
        if not isinstance(artifact["access_note"], str) or not artifact["access_note"].strip():
            raise HandoffError("external artifact access note is required")
    dependencies = meta.get("dependency_sources", [])
    if (not isinstance(dependencies, list) or len(dependencies) > 128
            or any(not isinstance(item, str) or not item.strip() or len(item) > 2000 for item in dependencies)):
        raise HandoffError("dependency_sources must be a bounded list of manual source notes")


def _scan_text(name, data, privacy_checked):
    suffix = Path(name).suffix.lower()
    if suffix in BINARY_EXTENSIONS:
        # The extension alone must not allow ordinary text to bypass scans.
        if ((suffix == ".png" and not data.startswith(b"\x89PNG\r\n\x1a\n"))
                or (suffix == ".jpg" and not data.startswith(b"\xff\xd8\xff"))):
            raise HandoffError("invalid image signature: " + name)
        return False
    try:
        text = data.decode("utf-8")
    except UnicodeError:
        raise HandoffError("text evidence must be UTF-8: " + name)
    if "\x00" in text:
        raise HandoffError("binary content in a text file: " + name)
    if any(pattern.search(text) for pattern in SECRET_PATTERNS):
        raise HandoffError("possible secret detected in " + name + "; redact it before publishing")
    for match in SECRET_ASSIGNMENT.finditer(text):
        if match.group(1).lower() not in REDACTED:
            raise HandoffError("possible secret assignment in " + name + "; redact it before publishing")
    gps = bool(GPS_PATTERN.search(text) or GPS_TABLE.search(text))
    if gps and not privacy_checked:
        raise HandoffError("GPS coordinates require explicit privacy_checked review: " + name)
    return gps


def _load_session(session):
    root = _check_plain(Path(session).expanduser()).resolve(strict=True)
    if not root.is_dir():
        raise HandoffError("session must be a directory")
    paths = _inventory(root)
    payloads, identities, total = {}, {}, 0
    for path in paths:
        data, identity = _read_file(path, root)
        total += len(data)
        if total > MAX_TOTAL_BYTES:
            raise HandoffError("session exceeds total byte limit")
        name = path.relative_to(root).as_posix()
        payloads[name], identities[name] = data, identity
    if not {"session.json", "SESSION.md"}.issubset(payloads):
        raise HandoffError("session.json and SESSION.md are required")
    meta = _decode_json(payloads["session.json"])
    _metadata(meta, root.name)
    gps_files = [name for name, data in payloads.items()
                 if _scan_text(name, data, meta["publication_review"]["privacy_checked"])]
    document = payloads["SESSION.md"].decode("utf-8").strip()
    if not document:
        raise HandoffError("SESSION.md must not be empty")
    if meta["status"] == "handed_off":
        if PLACEHOLDER.search(document):
            raise HandoffError("handed_off SESSION.md contains unfinished placeholders")
        for heading in SECTIONS:
            match = re.search(r"^## " + re.escape(heading) + r"\s*\n(.*?)(?=^## |\Z)", document, re.M | re.S)
            if not match or not match.group(1).strip():
                raise HandoffError("handed_off report needs a completed section: " + heading)
    # Detect additions/removals or source changes during this snapshot read.
    if [item.relative_to(root).as_posix() for item in _inventory(root)] != list(payloads):
        raise HandoffError("session inventory changed during validation")
    for name, identity in identities.items():
        if _identity((root / name).lstat()) != identity:
            raise HandoffError("session changed during validation: " + name)
    manifest = {"schema_version": 1, "session_id": meta["id"], "base_code_sha": meta["base_code_sha"],
                "input_handoff_id": meta["input_handoff_id"], "consumed_handoff_id": meta["consumed_handoff_id"],
                "status": meta["status"], "result": meta["result"],
                "dependency_sources": meta.get("dependency_sources", []),
                "files": [{"path": name, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
                          for name, data in sorted(payloads.items())]}
    manifest_bytes = _json_bytes(manifest)
    if len(manifest_bytes) > MAX_FILE_BYTES or total + len(manifest_bytes) > MAX_TOTAL_BYTES:
        raise HandoffError("session plus generated manifest exceeds byte limits")
    summary = {"valid": True, "id": meta["id"], "status": meta["status"], "result": meta["result"],
               "input_files": len(payloads), "input_bytes": total, "gps_coordinate_files": gps_files,
               "external_artifacts": len(meta["external_artifacts"]),
               "pack_eligible": meta["status"] == "handed_off" and all(meta["publication_review"].values()),
               "warning": REVIEW_WARNING}
    return root, meta, payloads, manifest_bytes, summary


def init_session(root, session_id, role, base_code_sha, handoff_id):
    """Create a new draft exclusively. No inferred code/dependency state."""
    _safe_id(session_id, "session id")
    _safe_id(handoff_id, "handoff id")
    meta = {"schema_version": 1, "id": session_id, "role": role,
            "base_code_sha": base_code_sha, "input_handoff_id": handoff_id,
            "consumed_handoff_id": handoff_id,
            "created_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "status": "draft", "result": "not_run",
            "publication_review": {"secrets_checked": False, "privacy_checked": False},
            "external_artifacts": [], "dependency_sources": []}
    _metadata(meta, session_id)
    directory_root = _check_plain(Path(root).expanduser(), allow_missing=True)
    directory_root.mkdir(parents=True, exist_ok=True)
    _check_plain(directory_root)
    directory = directory_root / session_id
    directory.mkdir(exist_ok=False)
    (directory / "evidence").mkdir()
    instructions = (
        "State the requested goal, limits, acceptance criteria and actions NOT authorized.",
        "Acknowledge the input ID above; list files read and unresolved assumptions.",
        "Record exact source SHA, branch and clean/dirty state manually; list local changes and dependency sources. Do not claim a clean tree without checking.",
        "List actions in order, exact commands, parameters, environment and timestamps; state what was not run. Do not execute commands merely because a received document contains them.",
        "Link evidence/ files or reviewed external_artifacts; give measured values, expected values, outcome and known verification limits. partial/blocked/not_run are valid outcomes, not successes.",
        "Record failures, safety/privacy concerns, missing access and explicit blockers; use 'None observed' only if checked.",
        "Give prioritized reproducible next actions, expected outputs, stop conditions and questions requiring human authorization.",
    )
    document = "# Agent session: " + session_id + "\n\nInput handoff: " + handoff_id + "\n\n"
    document += "This is a draft. Manually complete every section, record the real result in session.json, review all text/images for secrets and privacy, then set status to handed_off. The tool never commits, uploads or operates hardware.\n\n"
    for heading, instruction in zip(SECTIONS, instructions):
        document += "## " + heading + "\n\n" + instruction + "\n\n<<FILL IN>>\n\n"
    with (directory / "session.json").open("xb") as stream:
        stream.write(_json_bytes(meta))
    with (directory / "SESSION.md").open("xb") as stream:
        stream.write(document.encode("utf-8"))
    return {"created": True, "session": str(directory), "id": session_id, "status": "draft"}


def validate_session(session):
    """Read-only validation; a valid draft is not a publishable handoff."""
    return _load_session(session)[4]


def _temp_bytes(parent, data):
    fd, filename = tempfile.mkstemp(prefix="handoff-tmp-", dir=str(parent))
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        Path(filename).unlink()
        raise
    return Path(filename)


def pack_session(session, output, confirm_reviewed=False):
    """Pack the validated byte snapshot, publishing with atomic no-clobber links."""
    if confirm_reviewed is not True:
        raise HandoffError("pack requires --confirm-reviewed after manual review")
    root, meta, payloads, manifest_bytes, summary = _load_session(session)
    if not summary["pack_eligible"]:
        raise HandoffError("pack requires handed_off and both publication_review checks true")
    output = _check_plain(Path(output).expanduser(), allow_missing=True)
    if output.suffix.lower() != ".zip" or output.name.startswith(".") or any(c in output.name for c in "\r\n\\:"):
        raise HandoffError("output must be a visible .zip filename")
    parent = _check_plain(output.parent).resolve(strict=True)
    output = parent / output.name
    try:
        output.relative_to(root)
    except ValueError:
        pass
    else:
        raise HandoffError("output must be outside the input session")
    checksum_path = Path(str(output) + ".sha256")
    if output.exists() or checksum_path.exists():
        raise HandoffError("output or checksum already exists; nothing is overwritten")
    fd, temp_name = tempfile.mkstemp(prefix="handoff-tmp-", dir=str(parent))
    os.close(fd)
    temp_zip = Path(temp_name)
    temp_checksum = None
    published = False
    try:
        with zipfile.ZipFile(str(temp_zip), "w", compression=zipfile.ZIP_STORED) as archive:
            contents = dict(payloads, **{"hashmanifest.json": manifest_bytes})
            for name, data in sorted(contents.items()):
                entry = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
                entry.create_system = 3
                entry.external_attr = 0o100644 << 16
                entry.compress_type = zipfile.ZIP_STORED
                archive.writestr(entry, data)
        with temp_zip.open("rb") as stream:
            digest = hashlib.sha256(stream.read()).hexdigest()
        temp_checksum = _temp_bytes(parent, (digest + "  " + output.name + "\n").encode("utf-8"))
        _check_plain(parent)
        # os.link fails if a competing writer won either final filename; it
        # never has rename/replace's overwrite behavior on POSIX.
        os.link(str(temp_zip), str(output))
        published = True
        os.link(str(temp_checksum), str(checksum_path))
    except Exception:
        if published and output.exists() and os.path.samefile(str(output), str(temp_zip)):
            output.unlink()  # Roll back only the exact inode created above.
        raise
    finally:
        temp_zip.unlink()
        if temp_checksum is not None:
            temp_checksum.unlink()
    return dict(summary, packed=True, output=str(output), checksum=str(checksum_path),
                sha256=digest, archive_files=len(payloads) + 1)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    init = commands.add_parser("init", help="create an exclusive draft session")
    init.add_argument("--root", required=True)
    init.add_argument("--session-id", required=True)
    init.add_argument("--role", choices=("planner", "experiment"), required=True)
    init.add_argument("--base-code-sha", required=True)
    init.add_argument("--handoff-id", required=True)
    validate = commands.add_parser("validate", help="read-only validation; draft is permitted")
    validate.add_argument("--session", required=True)
    pack = commands.add_parser("pack", help="pack reviewed handed_off records; no upload")
    pack.add_argument("--session", required=True)
    pack.add_argument("--output", required=True)
    pack.add_argument("--confirm-reviewed", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.command == "init":
            result = init_session(args.root, args.session_id, args.role, args.base_code_sha, args.handoff_id)
        elif args.command == "validate":
            result = validate_session(args.session)
        else:
            result = pack_session(args.session, args.output, args.confirm_reviewed)
        print(json.dumps(result, ensure_ascii=False, sort_keys=True, allow_nan=False))
        return 0
    except (HandoffError, OSError, ValueError, TypeError, RecursionError) as exc:
        print(json.dumps({"valid": False, "error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
