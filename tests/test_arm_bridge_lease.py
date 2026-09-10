import os

import pytest

from so101_arm_bridge.backend_lease import BackendLeaseError, acquire_backend_lease
from so101_arm_bridge.wire import cobs_decode, cobs_encode, crc32c
from tools.lib import actuator_protocol


def test_one_process_cannot_bypass_existing_owner_with_a_different_domain(tmp_path):
    with acquire_backend_lease("stm32", 0, tmp_path):
        with pytest.raises(BackendLeaseError):
            acquire_backend_lease("stm32", 1, tmp_path)
    with acquire_backend_lease("stm32", 1, tmp_path) as lease:
        assert lease.acquired


def test_old_parent_lease_variable_does_not_grant_a_second_serial_owner(tmp_path, monkeypatch):
    with acquire_backend_lease("stm32", 0, tmp_path):
        monkeypatch.setenv("SO101_BACKEND_LEASE_OWNER_PID", str(os.getpid()))
        with pytest.raises(BackendLeaseError):
            acquire_backend_lease("stm32", 0, tmp_path)


@pytest.mark.parametrize("backend", ["mock", "isaac", "unknown"])
def test_removed_backends_cannot_acquire_the_hardware_lease(tmp_path, backend):
    with pytest.raises(BackendLeaseError):
        acquire_backend_lease(backend, 0, tmp_path)


@pytest.mark.parametrize("domain", [True, -1, 233, 1.5])
def test_invalid_ros_domain_does_not_create_a_lease(tmp_path, domain):
    with pytest.raises(BackendLeaseError):
        acquire_backend_lease("stm32", domain, tmp_path)
    assert not list(tmp_path.iterdir())


def test_extracted_wire_codec_matches_the_preserved_mcu_protocol():
    assert crc32c(b"123456789") == 0xE3069283
    for payload in (b"", b"\0", bytes(range(256)), b"x" * 512):
        assert crc32c(payload) == actuator_protocol.crc32c(payload)
        assert cobs_encode(payload) == actuator_protocol.cobs_encode(payload)
        assert cobs_decode(cobs_encode(payload)) == payload
