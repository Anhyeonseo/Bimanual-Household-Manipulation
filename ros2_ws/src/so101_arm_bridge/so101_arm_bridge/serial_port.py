"""Exclusive serial-port construction for the STM32 backend."""

from __future__ import annotations

from typing import Any


def open_exclusive_serial(
    serial_module: Any,
    device: str,
    baud_rate: int,
    timeout_s: float,
) -> Any:
    """Open a POSIX-exclusive pyserial session."""

    return serial_module.Serial(
        device,
        baud_rate,
        timeout=timeout_s,
        write_timeout=timeout_s,
        exclusive=True,
    )
