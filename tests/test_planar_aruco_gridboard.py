import importlib.util
import sys
import unittest
from pathlib import Path

import cv2


TOOLS = Path(__file__).resolve().parents[1] / "tools/setup/camera_calibration"
SPEC = importlib.util.spec_from_file_location(
    "generate_planar_aruco_gridboard",
    TOOLS / "generate_planar_aruco_gridboard.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class PlanarArucoGridBoardTest(unittest.TestCase):
    def test_board_contract_and_marker_ids(self):
        self.assertAlmostEqual(MODULE.BOARD_WIDTH_M, 0.095)
        self.assertAlmostEqual(MODULE.BOARD_HEIGHT_M, 0.120)
        self.assertAlmostEqual(MODULE.MARKER_LENGTH_M, 0.020)
        self.assertAlmostEqual(MODULE.MARKER_SEPARATION_M, 0.005)
        board = MODULE.create_board()
        self.assertEqual(
            board.ids.reshape(-1).tolist(),
            list(range(10, 30)),
        )

    def test_generated_board_detects_all_twenty_ids(self):
        board_image = MODULE.draw_board(1900)
        quiet = cv2.copyMakeBorder(
            board_image,
            200,
            200,
            200,
            200,
            cv2.BORDER_CONSTANT,
            value=255,
        )
        dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        _, ids, _ = cv2.aruco.detectMarkers(quiet, dictionary)
        self.assertIsNotNone(ids)
        self.assertEqual(
            sorted(ids.reshape(-1).tolist()),
            list(range(10, 30)),
        )


if __name__ == "__main__":
    unittest.main()
