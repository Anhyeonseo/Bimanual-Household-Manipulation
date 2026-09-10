"""Process-lifetime ownership guard for the SO-101 backend."""

from __future__ import annotations

import fcntl
import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import IO


SUPPORTED_BACKENDS = frozenset({"stm32"})
LOCK_FILENAME = "so101-backend.lock"


class BackendLeaseError(RuntimeError):
    """Raised when a backend name or ownership request is invalid."""


def _runtime_directory(explicit_directory: Path | None = None) -> Path:
    if explicit_directory is not None:
        return explicit_directory

    xdg_runtime_directory = os.environ.get("XDG_RUNTIME_DIR")
    if xdg_runtime_directory:
        return Path(xdg_runtime_directory)

    standard_directory = Path("/run/user") / str(os.getuid())
    if standard_directory.is_dir():
        return standard_directory

    return Path(tempfile.gettempdir()) / f"so101-runtime-{os.getuid()}"


def _validate_backend(backend: str) -> str:
    normalized = backend.strip().lower()
    if normalized not in SUPPORTED_BACKENDS:
        supported = ", ".join(sorted(SUPPORTED_BACKENDS))
        raise BackendLeaseError(
            f"unsupported backend {backend!r}; expected one of: {supported}"
        )
    return normalized


def _validate_domain_id(ros_domain_id: int) -> int:
    if (
        isinstance(ros_domain_id, bool)
        or not isinstance(ros_domain_id, int)
        or not 0 <= ros_domain_id <= 232
    ):
        raise BackendLeaseError("ROS domain ID must be an integer within 0..232")
    return ros_domain_id


@dataclass
class BackendLease:
    """An exclusive file lock held until release or process exit."""

    backend: str
    ros_domain_id: int
    path: Path
    _stream: IO[str] | None

    @property
    def acquired(self) -> bool:
        return self._stream is not None

    def release(self) -> None:
        stream = self._stream
        if stream is None:
            return
        self._stream = None
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
        finally:
            stream.close()

    def __enter__(self) -> BackendLease:
        """Return the acquired lease for context-manager use."""
        return self

    def __exit__(self, *unused: object) -> None:
        """Release the lease when leaving a context."""
        self.release()


def acquire_backend_lease(
    backend: str,
    ros_domain_id: int,
    runtime_directory: Path | None = None,
) -> BackendLease:
    """Acquire the single SO-101 backend lease without waiting."""
    validated_backend = _validate_backend(backend)
    validated_domain_id = _validate_domain_id(ros_domain_id)
    directory = _runtime_directory(runtime_directory)
    lock_path = directory / LOCK_FILENAME

    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    descriptor = os.open(lock_path, os.O_RDWR | os.O_CREAT, 0o600)
    stream = os.fdopen(descriptor, "r+", encoding="utf-8")

    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        stream.seek(0)
        holder = stream.read().strip() or "owner details unavailable"
        stream.close()
        raise BackendLeaseError(
            f"SO-101 backend is already active: {holder}"
        ) from error
    except Exception:
        stream.close()
        raise

    owner = {
        "backend": validated_backend,
        "pid": os.getpid(),
        "ros_domain_id": validated_domain_id,
    }
    try:
        stream.seek(0)
        stream.truncate()
        json.dump(owner, stream, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    except Exception:
        stream.close()
        raise
    return BackendLease(
        backend=validated_backend,
        ros_domain_id=validated_domain_id,
        path=lock_path,
        _stream=stream,
    )
