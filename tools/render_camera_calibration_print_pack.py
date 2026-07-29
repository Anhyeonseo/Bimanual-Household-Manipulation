#!/usr/bin/env python3
"""Render the SO-101 A4 camera-calibration print pack at exact dimensions."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Callable

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


CHECKER_INNER_CORNERS = (9, 6)
CHECKER_SQUARE_MM = 25.0
CHECKER_SQUARES = (
    CHECKER_INNER_CORNERS[0] + 1,
    CHECKER_INNER_CORNERS[1] + 1,
)

TCP_BOARD_MM = (45.0, 45.0)
TCP_QUIET_MM = 10.0
TCP_GRIP_TAB_MM = 20.0

PLANAR_BOARD_MM = (95.0, 120.0)
PLANAR_QUIET_MM = 10.0

VALIDATION_GRID_MM = (180.0, 240.0)
VALIDATION_CELL_MM = 20.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tcp-png", required=True, type=Path)
    parser.add_argument("--planar-png", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    return parser.parse_args()


def draw_scale_bar(
    pdf: canvas.Canvas,
    x: float,
    y: float,
    length_mm: float,
    label: str | None = None,
) -> None:
    length = length_mm * mm
    pdf.setStrokeColor(colors.black)
    pdf.setFillColor(colors.black)
    pdf.setLineWidth(0.45)
    pdf.line(x, y, x + length, y)
    pdf.line(x, y - 2.0 * mm, x, y + 2.0 * mm)
    pdf.line(x + length, y - 2.0 * mm, x + length, y + 2.0 * mm)
    pdf.setFont("Helvetica", 8)
    pdf.drawCentredString(
        x + length / 2.0,
        y + 3.0 * mm,
        label or f"VERIFY EXACTLY {length_mm:.2f} mm",
    )


def draw_page_header(
    pdf: canvas.Canvas,
    page_width: float,
    page_height: float,
    title: str,
    subtitle: str,
) -> None:
    pdf.setFillColor(colors.black)
    pdf.setFont("Helvetica-Bold", 14)
    pdf.drawCentredString(
        page_width / 2.0,
        page_height - 15.0 * mm,
        title,
    )
    pdf.setFont("Helvetica", 8.5)
    pdf.drawCentredString(
        page_width / 2.0,
        page_height - 21.0 * mm,
        subtitle,
    )


def draw_index_page(pdf: canvas.Canvas) -> None:
    page_width, page_height = A4
    pdf.setPageSize(A4)
    draw_page_header(
        pdf,
        page_width,
        page_height,
        "SO-101 CAMERA CALIBRATION PRINT PACK",
        "A4 | ACTUAL SIZE / 100% | DO NOT FIT, SHRINK, OR SCALE",
    )

    left = 22.0 * mm
    y = page_height - 38.0 * mm
    pdf.setFont("Helvetica-Bold", 11)
    pdf.drawString(left, y, "PRINT CONTRACT")
    pdf.setFont("Helvetica", 9)
    contract = [
        "1. Select Actual size or 100% in the print dialog.",
        "2. Disable Fit to page, Shrink oversized pages, and borderless scaling.",
        "3. Use A4 paper. Auto portrait/landscape orientation is allowed.",
        "4. Measure each verification bar before using a target.",
        "5. Reprint if the measured error exceeds 0.5 mm over 100 mm.",
    ]
    for line in contract:
        y -= 6.0 * mm
        pdf.drawString(left, y, line)

    y -= 10.0 * mm
    pdf.setFont("Helvetica-Bold", 11)
    pdf.drawString(left, y, "PAGES AND USE")
    entries = [
        (
            "2  CHECKERBOARD",
            "9x6 inner corners, 25 mm squares. Top/wrist camera intrinsics.",
        ),
        (
            "3  TCP GRIDBOARD",
            "IDs 0-3. Grip tab target for top eye-to-hand calibration.",
        ),
        (
            "4  PLANAR GRIDBOARD",
            "IDs 10-29. Fixed flat target for wrist/extrinsic pose checks.",
        ),
        (
            "5  VALIDATION GRID",
            "20 mm metric grid for independent tabletop x/y/yaw tests.",
        ),
    ]
    for heading, description in entries:
        y -= 8.0 * mm
        pdf.setFont("Helvetica-Bold", 9)
        pdf.drawString(left, y, heading)
        pdf.setFont("Helvetica", 8.5)
        pdf.drawString(left + 40.0 * mm, y, description)

    y -= 14.0 * mm
    pdf.setFont("Helvetica-Bold", 11)
    pdf.drawString(left, y, "MOUNTING")
    mounting = [
        "Checkerboard: keep flat; mount to foam board if paper curls.",
        "TCP GridBoard: cut dashed card and glue to a flat rigid card.",
        "Planar GridBoard: keep the full white quiet zone; mount perfectly flat.",
        "Prefer matte paper or matte adhesive. Avoid glossy lamination and glare.",
        "Do not mix the 25 mm checkerboard with old 30.45 mm homography samples.",
    ]
    pdf.setFont("Helvetica", 9)
    for line in mounting:
        y -= 6.0 * mm
        pdf.drawString(left, y, line)

    draw_scale_bar(
        pdf,
        (page_width - 100.0 * mm) / 2.0,
        35.0 * mm,
        100.0,
    )
    pdf.setFont("Helvetica-Bold", 9)
    pdf.drawCentredString(
        page_width / 2.0,
        20.0 * mm,
        "PRINTING TARGETS DOES NOT AUTHORIZE ROBOT MOTION.",
    )


def draw_checkerboard_page(pdf: canvas.Canvas) -> None:
    page_size = landscape(A4)
    page_width, page_height = page_size
    pdf.setPageSize(page_size)

    squares_x, squares_y = CHECKER_SQUARES
    board_width = squares_x * CHECKER_SQUARE_MM * mm
    board_height = squares_y * CHECKER_SQUARE_MM * mm
    board_x = (page_width - board_width) / 2.0
    board_y = (page_height - board_height) / 2.0

    pdf.setFillColor(colors.black)
    pdf.setFont("Helvetica-Bold", 8)
    pdf.drawCentredString(
        page_width / 2.0,
        page_height - 6.0 * mm,
        "CAMERA INTRINSICS - 9x6 INNER CORNERS - 25.00 mm SQUARES",
    )
    pdf.setFont("Helvetica", 7)
    pdf.drawCentredString(
        page_width / 2.0,
        page_height - 11.0 * mm,
        "USE square_size=0.025 m | PRINT A4 LANDSCAPE AT ACTUAL SIZE / 100%",
    )

    for row in range(squares_y):
        for column in range(squares_x):
            if (row + column) % 2 == 0:
                pdf.setFillColor(colors.black)
            else:
                pdf.setFillColor(colors.white)
            pdf.rect(
                board_x + column * CHECKER_SQUARE_MM * mm,
                board_y + (squares_y - 1 - row) * CHECKER_SQUARE_MM * mm,
                CHECKER_SQUARE_MM * mm,
                CHECKER_SQUARE_MM * mm,
                stroke=0,
                fill=1,
            )
    pdf.setStrokeColor(colors.black)
    pdf.setLineWidth(0.35)
    pdf.rect(
        board_x,
        board_y,
        board_width,
        board_height,
        stroke=1,
        fill=0,
    )
    draw_scale_bar(
        pdf,
        (page_width - 100.0 * mm) / 2.0,
        7.0 * mm,
        100.0,
    )


def draw_tcp_gridboard_page(
    pdf: canvas.Canvas,
    tcp_png: Path,
) -> None:
    page_width, page_height = A4
    pdf.setPageSize(A4)
    draw_page_header(
        pdf,
        page_width,
        page_height,
        "TOP EYE-TO-HAND TCP GRIDBOARD",
        "DICT_4X4_50 IDs 0-3 | 20 mm MARKERS | 5 mm GAP",
    )

    card_width_mm = (
        TCP_BOARD_MM[0] + 2.0 * TCP_QUIET_MM
    )
    card_height_mm = (
        TCP_BOARD_MM[1]
        + 2.0 * TCP_QUIET_MM
        + TCP_GRIP_TAB_MM
    )
    card_width = card_width_mm * mm
    card_height = card_height_mm * mm
    card_x = (page_width - card_width) / 2.0
    card_y = 145.0 * mm

    pdf.setDash(3, 2)
    pdf.setLineWidth(0.4)
    pdf.rect(card_x, card_y, card_width, card_height)
    pdf.setDash()

    board_x = card_x + TCP_QUIET_MM * mm
    board_y = (
        card_y
        + TCP_QUIET_MM * mm
        + TCP_GRIP_TAB_MM * mm
    )
    pdf.drawImage(
        str(tcp_png),
        board_x,
        board_y,
        width=TCP_BOARD_MM[0] * mm,
        height=TCP_BOARD_MM[1] * mm,
        preserveAspectRatio=True,
        mask="auto",
    )

    tab_top = card_y + TCP_GRIP_TAB_MM * mm
    pdf.setLineWidth(0.25)
    pdf.line(card_x, tab_top, card_x + card_width, tab_top)
    pdf.setFont("Helvetica-Bold", 9)
    pdf.drawCentredString(
        card_x + card_width / 2.0,
        card_y + 11.5 * mm,
        "GRIP ONLY",
    )
    pdf.setFont("Helvetica", 7)
    pdf.drawCentredString(
        card_x + card_width / 2.0,
        card_y + 6.5 * mm,
        "KEEP ALL FOUR MARKERS VISIBLE",
    )

    draw_scale_bar(
        pdf,
        (page_width - 50.0 * mm) / 2.0,
        125.0 * mm,
        50.0,
    )
    pdf.setFont("Helvetica", 8.5)
    instructions = [
        "Cut on the dashed outline and glue the complete card to rigid flat stock.",
        "The gripper touches only the tab; never cover, bend, or trim the marker grid.",
        "Use for 8 training poses plus 2 independent validation poses.",
        "Calibration output remains motion_authorized=false until validation passes.",
    ]
    start_y = 105.0 * mm
    for index, line in enumerate(instructions):
        pdf.drawString(
            25.0 * mm,
            start_y - index * 6.0 * mm,
            line,
        )


def draw_planar_gridboard_page(
    pdf: canvas.Canvas,
    planar_png: Path,
) -> None:
    page_width, page_height = A4
    pdf.setPageSize(A4)
    draw_page_header(
        pdf,
        page_width,
        page_height,
        "PLANAR CAMERA POSE GRIDBOARD",
        "DICT_4X4_50 IDs 10-29 | 4x5 | 20 mm MARKERS | 5 mm GAP",
    )

    card_width_mm = PLANAR_BOARD_MM[0] + 2.0 * PLANAR_QUIET_MM
    card_height_mm = PLANAR_BOARD_MM[1] + 2.0 * PLANAR_QUIET_MM
    card_width = card_width_mm * mm
    card_height = card_height_mm * mm
    card_x = (page_width - card_width) / 2.0
    card_y = 87.0 * mm

    pdf.setDash(3, 2)
    pdf.setLineWidth(0.4)
    pdf.rect(card_x, card_y, card_width, card_height)
    pdf.setDash()
    pdf.drawImage(
        str(planar_png),
        card_x + PLANAR_QUIET_MM * mm,
        card_y + PLANAR_QUIET_MM * mm,
        width=PLANAR_BOARD_MM[0] * mm,
        height=PLANAR_BOARD_MM[1] * mm,
        preserveAspectRatio=True,
        mask="auto",
    )

    draw_scale_bar(
        pdf,
        (page_width - 100.0 * mm) / 2.0,
        69.0 * mm,
        100.0,
    )
    pdf.setFont("Helvetica", 8.5)
    instructions = [
        "Keep the entire dashed card, including the 10 mm white quiet zone.",
        "Mount to rigid flat stock. Do not bend, crop, or use glossy laminate.",
        "Use as a fixed target for wrist-camera intrinsics/extrinsic pose checks.",
        "This board uses IDs 10-29 and cannot be confused with TCP IDs 0-3.",
    ]
    start_y = 50.0 * mm
    for index, line in enumerate(instructions):
        pdf.drawString(
            25.0 * mm,
            start_y - index * 6.0 * mm,
            line,
        )


def draw_crosshair(
    pdf: canvas.Canvas,
    x: float,
    y: float,
    label: str,
) -> None:
    radius = 3.0 * mm
    arm = 5.0 * mm
    pdf.setStrokeColor(colors.black)
    pdf.setLineWidth(0.5)
    pdf.circle(x, y, radius, stroke=1, fill=0)
    pdf.line(x - arm, y, x + arm, y)
    pdf.line(x, y - arm, x, y + arm)
    pdf.setFillColor(colors.black)
    pdf.setFont("Helvetica", 6)
    pdf.drawString(x + 4.0 * mm, y + 3.0 * mm, label)


def draw_validation_grid_page(pdf: canvas.Canvas) -> None:
    page_width, page_height = A4
    pdf.setPageSize(A4)

    grid_width = VALIDATION_GRID_MM[0] * mm
    grid_height = VALIDATION_GRID_MM[1] * mm
    grid_x = (page_width - grid_width) / 2.0
    grid_y = (page_height - grid_height) / 2.0
    center_x = grid_x + grid_width / 2.0
    center_y = grid_y + grid_height / 2.0

    pdf.setFillColor(colors.black)
    pdf.setFont("Helvetica-Bold", 9)
    pdf.drawCentredString(
        page_width / 2.0,
        page_height - 7.0 * mm,
        "TABLETOP VALIDATION GRID - 20.00 mm CELLS",
    )
    pdf.setFont("Helvetica", 7)
    pdf.drawCentredString(
        page_width / 2.0,
        page_height - 12.0 * mm,
        "VALIDATION ONLY | ORIGIN IS PAGE CENTER | ACTUAL SIZE / 100%",
    )

    columns = round(VALIDATION_GRID_MM[0] / VALIDATION_CELL_MM)
    rows = round(VALIDATION_GRID_MM[1] / VALIDATION_CELL_MM)
    for column in range(columns + 1):
        x = grid_x + column * VALIDATION_CELL_MM * mm
        if column == columns // 2:
            pdf.setStrokeColor(colors.black)
            pdf.setLineWidth(0.8)
        else:
            pdf.setStrokeColor(colors.Color(0.72, 0.72, 0.72))
            pdf.setLineWidth(0.25)
        pdf.line(x, grid_y, x, grid_y + grid_height)
    for row in range(rows + 1):
        y = grid_y + row * VALIDATION_CELL_MM * mm
        if row == rows // 2:
            pdf.setStrokeColor(colors.black)
            pdf.setLineWidth(0.8)
        else:
            pdf.setStrokeColor(colors.Color(0.72, 0.72, 0.72))
            pdf.setLineWidth(0.25)
        pdf.line(grid_x, y, grid_x + grid_width, y)

    pdf.setStrokeColor(colors.black)
    pdf.setLineWidth(0.5)
    pdf.rect(grid_x, grid_y, grid_width, grid_height, stroke=1, fill=0)

    for x_mm in (-60.0, 0.0, 60.0):
        for y_mm in (-80.0, 0.0, 80.0):
            draw_crosshair(
                pdf,
                center_x + x_mm * mm,
                center_y + y_mm * mm,
                f"({int(x_mm):+d},{int(y_mm):+d})",
            )

    arrow_length = 18.0 * mm
    pdf.setLineWidth(0.8)
    pdf.line(center_x, center_y, center_x + arrow_length, center_y)
    pdf.line(center_x, center_y, center_x, center_y + arrow_length)
    diagonal = arrow_length / math.sqrt(2.0)
    pdf.setDash(2, 2)
    pdf.line(
        center_x,
        center_y,
        center_x + diagonal,
        center_y + diagonal,
    )
    pdf.setDash()
    pdf.setFont("Helvetica-Bold", 7)
    pdf.drawString(center_x + arrow_length + 1.5 * mm, center_y, "+X / 0 deg")
    pdf.drawString(center_x + 1.5 * mm, center_y + arrow_length, "+Y / 90 deg")
    pdf.drawString(
        center_x + diagonal + 1.5 * mm,
        center_y + diagonal,
        "45 deg",
    )

    pdf.setFont("Helvetica", 7)
    pdf.drawString(
        15.0 * mm,
        10.0 * mm,
        "Place flat in the reachable workspace. Register its pose before using coordinates.",
    )
    pdf.drawRightString(
        page_width - 15.0 * mm,
        10.0 * mm,
        "Grid: 180 x 240 mm",
    )


PageDrawer = Callable[[canvas.Canvas], None]


def render_pdf(
    output_path: Path,
    pages: list[PageDrawer],
    title: str,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pdf = canvas.Canvas(str(output_path), pagesize=A4, pageCompression=0)
    pdf.setTitle(title)
    pdf.setAuthor("SO101-Bimanual-Manipulation")
    for draw_page in pages:
        draw_page(pdf)
        pdf.showPage()
    pdf.save()


def render_all(
    tcp_png: Path,
    planar_png: Path,
    output_dir: Path,
) -> list[Path]:
    if not tcp_png.is_file():
        raise FileNotFoundError(tcp_png)
    if not planar_png.is_file():
        raise FileNotFoundError(planar_png)

    tcp_page = lambda pdf: draw_tcp_gridboard_page(pdf, tcp_png)
    planar_page = lambda pdf: draw_planar_gridboard_page(pdf, planar_png)
    outputs = [
        output_dir / "so101_camera_calibration_print_pack_A4.pdf",
        output_dir / "so101_camera_calibration_print_instructions_A4.pdf",
        output_dir / "so101_camera_intrinsic_checkerboard_9x6_25mm.pdf",
        output_dir / "so101_top_eye_to_hand_tcp_gridboard_20mm.pdf",
        output_dir / "so101_camera_planar_gridboard_4x5_20mm.pdf",
        output_dir / "so101_tabletop_validation_grid_20mm.pdf",
    ]
    render_pdf(
        outputs[0],
        [
            draw_index_page,
            draw_checkerboard_page,
            tcp_page,
            planar_page,
            draw_validation_grid_page,
        ],
        "SO-101 Camera Calibration Print Pack",
    )
    render_pdf(outputs[1], [draw_index_page], "SO-101 Print Instructions")
    render_pdf(
        outputs[2],
        [draw_checkerboard_page],
        "SO-101 9x6 Checkerboard 25 mm",
    )
    render_pdf(
        outputs[3],
        [tcp_page],
        "SO-101 TCP GridBoard 20 mm",
    )
    render_pdf(
        outputs[4],
        [planar_page],
        "SO-101 Planar GridBoard 4x5 20 mm",
    )
    render_pdf(
        outputs[5],
        [draw_validation_grid_page],
        "SO-101 Tabletop Validation Grid 20 mm",
    )
    return outputs


def main() -> int:
    args = parse_args()
    outputs = render_all(
        args.tcp_png,
        args.planar_png,
        args.output_dir,
    )
    print(
        "CAMERA_CALIBRATION_PRINT_PACK_PASS "
        f"files={len(outputs)} combined_pages=5 "
        f"output={outputs[0]}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
