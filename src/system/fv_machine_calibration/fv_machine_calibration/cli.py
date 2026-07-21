"""Offline hand-eye solve command; never talks to or moves the robot."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile

from .model import CalibrationError, load_session
from .solver import result_document, solve_session


def write_new_atomic(path: Path, document: dict[str, object]) -> None:
    path = path.resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() or path.is_symlink():
        raise CalibrationError(f"refusing to overwrite existing proposal: {path}")
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=str(path.parent)
    )
    try:
        os.fchmod(file_descriptor, 0o600)
        with os.fdopen(file_descriptor, "w", encoding="utf-8") as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists() or path.is_symlink():
            raise CalibrationError(f"refusing to overwrite existing proposal: {path}")
        try:
            os.link(temporary_name, path)
        except FileExistsError as exc:
            raise CalibrationError(
                f"refusing to overwrite existing proposal: {path}"
            ) from exc
        os.unlink(temporary_name)
        directory_fd = os.open(str(path.parent), os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except Exception:
        try:
            os.unlink(temporary_name)
        except OSError:
            pass
        raise


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Solve a recorded Fluent Vision hand-eye session offline."
    )
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        session = load_session(args.session)
        result = solve_session(session)
        proposal = result_document(session, result)
        proposal["source_session_sha256"] = hashlib.sha256(
            args.session.read_bytes()
        ).hexdigest()
        write_new_atomic(args.output, proposal)
    except CalibrationError as exc:
        parser.error(str(exc))
    print(args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
