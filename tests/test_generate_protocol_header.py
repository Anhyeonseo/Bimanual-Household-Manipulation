import json
import unittest
from pathlib import Path

from tools.setup.firmware.generate_protocol_header import render_header


ROOT = Path(__file__).parents[1]
MANIFEST_PATH = ROOT / "protocol" / "message_ids.json"
HEADER_PATH = ROOT / "firmware" / "stm32_actuator" / "include" / "actuator_core" / "message_ids.h"


class GenerateProtocolHeaderTests(unittest.TestCase):
    def test_checked_in_header_matches_manifest(self):
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        self.assertEqual(HEADER_PATH.read_text(encoding="utf-8"), render_header(manifest))

    def test_messages_are_sorted_by_numeric_id(self):
        manifest = {
            "protocol_version": 1,
            "messages": [
                {"id": 2, "name": "SECOND"},
                {"id": 1, "name": "FIRST"},
            ],
        }
        rendered = render_header(manifest)
        self.assertLess(rendered.index("ACTUATOR_MSG_FIRST"), rendered.index("ACTUATOR_MSG_SECOND"))


if __name__ == "__main__":
    unittest.main()
