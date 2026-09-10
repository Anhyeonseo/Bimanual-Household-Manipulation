"""Portable discovery for the STM32 ST-LINK virtual serial port."""

from __future__ import annotations

from pathlib import Path


STLINK_PATTERN = "usb-STMicroelectronics_STLINK-V3_*-if02"


def resolve_serial_device(
    configured: str,
    by_id_directory: Path = Path("/dev/serial/by-id"),
    fallback_device: Path = Path("/dev/ttyACM0"),
) -> str:
    """Resolve ``auto`` without embedding a board serial in the repository."""

    value = configured.strip()
    if value and value.lower() != "auto":
        return value

    matches = sorted(
        path for path in by_id_directory.glob(STLINK_PATTERN) if path.exists()
    )
    if len(matches) == 1:
        return str(matches[0])
    if len(matches) > 1:
        devices = ", ".join(str(path) for path in matches)
        raise RuntimeError(
            "multiple ST-LINK serial devices found; set serial_device in "
            f"bridge.local.yaml: {devices}"
        )
    if fallback_device.exists():
        return str(fallback_device)
    raise RuntimeError(
        "no ST-LINK serial device found; connect the NUCLEO or set "
        "serial_device in bridge.local.yaml"
    )
