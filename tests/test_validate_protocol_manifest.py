import copy
import json
import unittest
from pathlib import Path

from tools.run.validate_protocol_manifest import validate_manifest


MANIFEST_PATH = Path(__file__).parents[1] / "protocol" / "message_ids.json"


def load_manifest():
    return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))


class ValidateProtocolManifestTests(unittest.TestCase):
    def test_repository_manifest_passes(self):
        self.assertEqual(validate_manifest(load_manifest()), [])

    def test_duplicate_id_fails(self):
        manifest = load_manifest()
        manifest["messages"][1]["id"] = manifest["messages"][0]["id"]
        self.assertTrue(any("duplicates" in error for error in validate_manifest(manifest)))

    def test_reserved_id_fails(self):
        manifest = load_manifest()
        manifest["messages"][0]["id"] = 64
        errors = validate_manifest(manifest)
        self.assertTrue(any("reserved" in error for error in errors))

    def test_missing_mandatory_message_fails(self):
        manifest = load_manifest()
        manifest["messages"] = [message for message in manifest["messages"] if message["name"] != "HEARTBEAT"]
        self.assertTrue(any("mandatory messages" in error for error in validate_manifest(manifest)))

    def test_estop_serial_message_is_forbidden(self):
        manifest = copy.deepcopy(load_manifest())
        manifest["messages"].append({"id": 23, "name": "ESTOP", "direction": "HOST_TO_MCU", "category": "state_control", "ack_required": True})
        self.assertTrue(any("forbidden" in error for error in validate_manifest(manifest)))


if __name__ == "__main__":
    unittest.main()
