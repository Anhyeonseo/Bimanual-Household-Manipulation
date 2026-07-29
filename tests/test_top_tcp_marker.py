import importlib.util
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


MODULE_PATH = Path("tools/detect_top_tcp_marker.py")
SPEC = importlib.util.spec_from_file_location("detect_top_tcp_marker", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class TopTcpMarkerTest(unittest.TestCase):
    @staticmethod
    def _image(rectangles: list[tuple[int, int, int, int]]) -> np.ndarray:
        image = np.full((120, 160, 3), 240, dtype=np.uint8)
        for x0, y0, x1, y1 in rectangles:
            cv2.rectangle(image, (x0, y0), (x1, y1), (0, 255, 255), -1)
        return image

    def test_one_marker_reports_center_fail_closed(self) -> None:
        result = MODULE.detect_yellow_marker(self._image([(60, 40, 80, 60)]))

        self.assertEqual(result["status"], "TOP_TCP_MARKER_PASS")
        self.assertAlmostEqual(result["center_px"][0], 70.0)
        self.assertAlmostEqual(result["center_px"][1], 50.0)
        self.assertFalse(result["motion_authorized"])
        self.assertFalse(result["robot_target_available"])

    def test_zero_markers_is_rejected(self) -> None:
        with self.assertRaisesRegex(MODULE.MarkerDetectionError, "detected 0"):
            MODULE.detect_yellow_marker(self._image([]))

    def test_two_markers_are_rejected(self) -> None:
        with self.assertRaisesRegex(MODULE.MarkerDetectionError, "detected 2"):
            MODULE.detect_yellow_marker(
                self._image([(20, 20, 40, 40), (100, 70, 120, 90)])
            )

    def test_border_clipped_marker_is_rejected(self) -> None:
        with self.assertRaisesRegex(MODULE.MarkerDetectionError, "border"):
            MODULE.detect_yellow_marker(self._image([(0, 40, 20, 60)]))

    def test_marker_at_exact_border_margin_is_rejected(self) -> None:
        with self.assertRaisesRegex(MODULE.MarkerDetectionError, "border"):
            MODULE.detect_yellow_marker(self._image([(3, 40, 23, 60)]))


if __name__ == "__main__":
    unittest.main()
