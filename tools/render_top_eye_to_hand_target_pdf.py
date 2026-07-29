#!/usr/bin/env python3
"""Lay out the eye-to-hand GridBoard raster at exact physical dimensions."""

from __future__ import annotations

import argparse
from pathlib import Path

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


BOARD_SIDE_MM = 45.0
QUIET_ZONE_MM = 10.0
GRIP_TAB_MM = 20.0
CARD_WIDTH_MM = BOARD_SIDE_MM + 2.0 * QUIET_ZONE_MM
CARD_HEIGHT_MM = BOARD_SIDE_MM + 2.0 * QUIET_ZONE_MM + GRIP_TAB_MM


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-png", required=True, type=Path)
    parser.add_argument("--output-pdf", required=True, type=Path)
    return parser.parse_args()


def draw_dimension_bar(pdf: canvas.Canvas, x: float, y: float) -> None:
    length = 50.0 * mm
    pdf.setLineWidth(0.4)
    pdf.line(x, y, x + length, y)
    pdf.line(x, y - 2.0 * mm, x, y + 2.0 * mm)
    pdf.line(x + length, y - 2.0 * mm, x + length, y + 2.0 * mm)
    pdf.setFont("Helvetica", 9)
    pdf.drawCentredString(
        x + length / 2.0,
        y + 3.0 * mm,
        "50.00 mm verification bar",
    )


def render(input_png: Path, output_pdf: Path) -> None:
    if not input_png.is_file():
        raise FileNotFoundError(input_png)
    output_pdf.parent.mkdir(parents=True, exist_ok=True)
    page_width, page_height = A4
    pdf = canvas.Canvas(str(output_pdf), pagesize=A4, pageCompression=0)
    pdf.setTitle("SO-101 Top Eye-to-Hand ArUco GridBoard")
    pdf.setAuthor("SO101-Bimanual-Manipulation")

    pdf.setFont("Helvetica-Bold", 15)
    pdf.drawCentredString(
        page_width / 2.0,
        page_height - 20.0 * mm,
        "SO-101 Top Eye-to-Hand Calibration Target",
    )
    pdf.setFont("Helvetica", 9)
    pdf.drawCentredString(
        page_width / 2.0,
        page_height - 27.0 * mm,
        "Print at Actual size / 100%. Disable Fit, Shrink, and Scale.",
    )

    card_width = CARD_WIDTH_MM * mm
    card_height = CARD_HEIGHT_MM * mm
    card_x = (page_width - card_width) / 2.0
    card_y = page_height - 125.0 * mm
    pdf.setDash(3, 2)
    pdf.setLineWidth(0.4)
    pdf.rect(card_x, card_y, card_width, card_height)
    pdf.setDash()

    board_x = card_x + QUIET_ZONE_MM * mm
    board_y = (
        card_y
        + QUIET_ZONE_MM * mm
        + GRIP_TAB_MM * mm
    )
    pdf.drawImage(
        str(input_png),
        board_x,
        board_y,
        width=BOARD_SIDE_MM * mm,
        height=BOARD_SIDE_MM * mm,
        preserveAspectRatio=True,
        mask="auto",
    )

    tab_top = card_y + GRIP_TAB_MM * mm
    pdf.setLineWidth(0.25)
    pdf.line(card_x, tab_top, card_x + card_width, tab_top)
    pdf.setFont("Helvetica-Bold", 8)
    pdf.drawCentredString(
        card_x + card_width / 2.0,
        card_y + 11.0 * mm,
        "GRIP ONLY",
    )
    pdf.setFont("Helvetica", 7)
    pdf.drawCentredString(
        card_x + card_width / 2.0,
        card_y + 6.5 * mm,
        "KEEP ALL 4 MARKERS VISIBLE",
    )

    bar_x = (page_width - 50.0 * mm) / 2.0
    draw_dimension_bar(pdf, bar_x, card_y - 18.0 * mm)

    instruction_y = card_y - 35.0 * mm
    pdf.setFont("Helvetica", 9)
    instructions = [
        "1. Print at Actual size / 100%.",
        "2. Measure the verification bar; it must be 50.00 mm.",
        "3. Cut on the dashed outline and glue to a flat rigid card.",
        "4. Grip only the bottom tab. Do not bend or cover the marker grid.",
        "5. Grid: DICT_4X4_50 IDs 0-3, markers 20 mm, gap 5 mm.",
    ]
    for index, text in enumerate(instructions):
        pdf.drawString(
            35.0 * mm,
            instruction_y - index * 5.0 * mm,
            text,
        )

    pdf.setFont("Helvetica-Oblique", 8)
    pdf.drawString(
        35.0 * mm,
        15.0 * mm,
        "Calibration output remains motion_authorized=false.",
    )
    pdf.showPage()
    pdf.save()


def main() -> int:
    args = parse_args()
    render(args.input_png, args.output_pdf)
    print(
        "TOP_EYE_TO_HAND_TARGET_PDF_PASS "
        f"board_side_mm={BOARD_SIDE_MM:.3f} "
        f"card_mm={CARD_WIDTH_MM:.1f}x{CARD_HEIGHT_MM:.1f} "
        f"output={args.output_pdf}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
