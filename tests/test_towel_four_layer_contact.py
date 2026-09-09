import numpy as np
import pytest

from tools.lib.towel_four_layer_contact import (
    FourLayerContactError,
    RegisteredFaceFrame,
    select_continuous_four_layer_pinch,
    select_continuous_single_sheet_pinch,
)


def face(center_x, normal_x):
    return RegisteredFaceFrame(
        center_m=(center_x, 0.0, 0.0),
        inward_normal=(normal_x, 0.0, 0.0),
        tangent_u=(0.0, 1.0, 0.0),
        tangent_v=(0.0, 0.0, 1.0),
    )


def folded_grid(*, second_layer_y=0.001):
    """A 4x4 topology whose two column-halves overlap in world space."""
    nodes = np.zeros((16, 3), dtype=float)
    for row in range(4):
        for column in range(4):
            index = row * 4 + column
            layer_column = column if column < 2 else column - 2
            nodes[index] = (
                -0.008 + 0.016 * layer_column,
                -0.002 + 0.0013 * row + (second_layer_y if column >= 2 else 0.0),
                0.0,
            )
    return nodes


def test_two_folded_surfaces_produce_four_physical_face_contacts():
    result = select_continuous_four_layer_pinch(
        folded_grid(),
        grid_side=4,
        fixed_face=face(-0.0065, 1.0),
        moving_face=face(0.0065, -1.0),
        face_size_m=0.006,
        inward_depth_m=0.003,
    )
    assert len(result.anchors) == 4
    assert {(item.s1_layer, item.face) for item in result.anchors} == {
        ("s1_first_half", "fixed"),
        ("s1_first_half", "moving"),
        ("s1_second_half", "fixed"),
        ("s1_second_half", "moving"),
    }
    assert result.support_vertex_indices
    assert result.to_dict()["physical_contact_count"] == 4


def test_missing_one_folded_surface_fails_closed():
    nodes = folded_grid(second_layer_y=0.050)
    with pytest.raises(FourLayerContactError, match="s1_second_half"):
        select_continuous_four_layer_pinch(
            nodes,
            grid_side=4,
            fixed_face=face(-0.0065, 1.0),
            moving_face=face(0.0065, -1.0),
            face_size_m=0.006,
            inward_depth_m=0.003,
        )


def test_contact_outside_finite_face_fails_even_when_between_jaws():
    nodes = folded_grid()
    nodes[:, 1] += 0.020
    with pytest.raises(FourLayerContactError):
        select_continuous_four_layer_pinch(
            nodes,
            grid_side=4,
            fixed_face=face(-0.0065, 1.0),
            moving_face=face(0.0065, -1.0),
            face_size_m=0.006,
            inward_depth_m=0.003,
        )


def test_coarse_single_sheet_triangle_contacts_both_faces_between_vertices():
    nodes = np.asarray(
        [
            (-0.008, -0.008, 0.0),
            (0.008, -0.008, 0.0),
            (-0.008, 0.008, 0.0),
            (0.008, 0.008, 0.0),
        ]
    )
    result = select_continuous_single_sheet_pinch(
        nodes,
        grid_side=2,
        fixed_face=face(-0.004, 1.0),
        moving_face=face(0.004, -1.0),
        face_size_m=0.006,
        inward_depth_m=0.003,
    )
    assert len(result.anchors) == 2
    assert {anchor.face for anchor in result.anchors} == {"fixed", "moving"}
    assert result.to_dict()["physical_contact_count"] == 2


def test_single_sheet_contact_outside_finite_face_fails_closed():
    nodes = np.asarray(
        [
            (-0.008, 0.020, 0.0),
            (0.008, 0.020, 0.0),
            (-0.008, 0.030, 0.0),
            (0.008, 0.030, 0.0),
        ]
    )
    with pytest.raises(FourLayerContactError, match="single sheet"):
        select_continuous_single_sheet_pinch(
            nodes,
            grid_side=2,
            fixed_face=face(-0.004, 1.0),
            moving_face=face(0.004, -1.0),
            face_size_m=0.006,
            inward_depth_m=0.003,
        )
