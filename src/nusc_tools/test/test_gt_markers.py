"""Tests for the GT markers published next to the noisy detections."""

import math

import pytest
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from nusc_tools.gt_publisher import (
    FRAME_ID, GT_RGBA, box_pose, box_to_marker, detections_to_msgs, gt_to_markers,
    stamp_from_seconds)
from nusc_tools.noise import SimDetection
from nusc_tools.scene_loader import GtBox, quat_from_yaw

TIMESTAMP_S = 1500000000.547590  # synthetic, nuScenes-like magnitude


def make_gt(name, x, y, size_wlh, yaw):
    return GtBox(
        instance_token=f'tok-{name}',
        tracking_name=name,
        translation=[x, y, 1.0],
        size_wlh=size_wlh,
        rotation_wxyz=quat_from_yaw(yaw),
        yaw=yaw,
        velocity_xy=[0.0, 0.0],
        num_pts=10,
    )


@pytest.fixture
def gt_boxes():
    # w, l, h all different so that any swap of the size fields is caught.
    return [
        make_gt('car', 100.0, 200.0, [1.8, 4.5, 1.5], 0.3),
        make_gt('pedestrian', 110.0, 230.0, [0.6, 0.7, 1.8], -2.0),
    ]


@pytest.fixture
def header():
    return Header(stamp=stamp_from_seconds(TIMESTAMP_S), frame_id=FRAME_ID)


# --- box_to_marker with an explicit color ----------------------------------

def test_box_to_marker_accepts_gt_box_with_rgba(gt_boxes, header):
    # A GtBox has no score: with rgba given, the score must not be read.
    marker = box_to_marker(gt_boxes[0], header, marker_id=3, ns='ground_truth', rgba=GT_RGBA)
    assert marker.ns == 'ground_truth'
    assert marker.id == 3


def test_box_to_marker_rgba_overrides_class_color(gt_boxes, header):
    marker = box_to_marker(gt_boxes[0], header, marker_id=0, rgba=(0.1, 0.2, 0.3, 0.4))
    # ColorRGBA stores float32, hence the tolerance.
    assert marker.color.r == pytest.approx(0.1, abs=1e-6)
    assert marker.color.g == pytest.approx(0.2, abs=1e-6)
    assert marker.color.b == pytest.approx(0.3, abs=1e-6)
    assert marker.color.a == pytest.approx(0.4, abs=1e-6)


def test_box_to_marker_default_unchanged(header):
    # Without rgba the old behavior stays: class color, opacity from the score.
    det = SimDetection(tracking_name='car', translation=[0.0, 0.0, 0.0],
                       size_wlh=[1.8, 4.5, 1.5], rotation_wxyz=[1.0, 0.0, 0.0, 0.0],
                       yaw=0.0, score=0.5)
    marker = box_to_marker(det, header, marker_id=0)
    assert marker.ns == 'detections'
    assert (marker.color.r, marker.color.g, marker.color.b) == (1.0, 1.0, 0.0)  # car = yellow
    assert marker.color.a == pytest.approx(0.75, abs=1e-6)


# --- gt_to_markers ---------------------------------------------------------

def test_gt_to_markers_counts_and_deleteall_first(gt_boxes):
    markers = gt_to_markers(TIMESTAMP_S, gt_boxes).markers
    assert len(markers) == len(gt_boxes) + 1
    assert markers[0].action == Marker.DELETEALL
    assert all(m.action == Marker.ADD for m in markers[1:])


def test_gt_to_markers_namespace_and_color(gt_boxes):
    for m in gt_to_markers(TIMESTAMP_S, gt_boxes).markers[1:]:
        assert m.ns == 'ground_truth'
        rgba = (m.color.r, m.color.g, m.color.b, m.color.a)
        assert rgba == pytest.approx(GT_RGBA, abs=1e-6)


def test_gt_to_markers_geometry(gt_boxes):
    markers = gt_to_markers(TIMESTAMP_S, gt_boxes).markers[1:]
    for marker, box in zip(markers, gt_boxes):
        assert marker.type == Marker.CUBE
        assert marker.pose == box_pose(box)
        # wlh -> (x = length, y = width, z = height)
        assert (marker.scale.x, marker.scale.y, marker.scale.z) == (
            box.size_wlh[1], box.size_wlh[0], box.size_wlh[2])


def test_gt_to_markers_ids_unique(gt_boxes):
    ids = [m.id for m in gt_to_markers(TIMESTAMP_S, gt_boxes).markers[1:]]
    assert len(set(ids)) == len(ids)


def test_gt_to_markers_empty_keyframe():
    # No GT box: the old GT markers must still be cleared in RViz.
    markers = gt_to_markers(TIMESTAMP_S, []).markers
    assert len(markers) == 1
    assert markers[0].action == Marker.DELETEALL


def test_gt_and_detections_share_the_header(gt_boxes):
    # Same keyframe -> same stamp and frame: RViz shows GT and noise together.
    det = SimDetection(tracking_name='car', translation=[100.0, 200.0, 1.0],
                       size_wlh=[1.8, 4.5, 1.5], rotation_wxyz=quat_from_yaw(0.3),
                       yaw=0.3, score=0.8)
    _, det_markers = detections_to_msgs(TIMESTAMP_S, [det])
    gt_markers = gt_to_markers(TIMESTAMP_S, gt_boxes)
    expected = Header(stamp=stamp_from_seconds(TIMESTAMP_S), frame_id=FRAME_ID)
    assert all(m.header == expected for m in det_markers.markers)
    assert all(m.header == expected for m in gt_markers.markers)


def test_gt_to_markers_does_not_modify_boxes(gt_boxes):
    before = [(list(b.translation), list(b.size_wlh), b.yaw) for b in gt_boxes]
    gt_to_markers(TIMESTAMP_S, gt_boxes)
    after = [(list(b.translation), list(b.size_wlh), b.yaw) for b in gt_boxes]
    assert before == after
    assert math.isclose(gt_boxes[0].yaw, 0.3)
