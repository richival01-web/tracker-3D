"""Tests for gt_publisher."""

import math

from geometry_msgs.msg import Vector3
import pytest
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

from nusc_tools.gt_publisher import (
    FRAME_ID, box_pose, box_to_detection3d, box_to_marker, keyframe_to_msgs,
    next_frame_index, stamp_from_seconds)
from nusc_tools.scene_loader import GtBox, Keyframe


def test_real_timestamp():
    stamp = stamp_from_seconds(1533151603.547590)
    assert stamp.sec == 1533151603
    assert abs(stamp.nanosec - 547590000) < 1000


def test_real_timestamp_small_fraction():
    stamp = stamp_from_seconds(1533151604.048025)
    assert stamp.sec == 1533151604
    assert abs(stamp.nanosec - 48025000) < 1000


def test_nanosec_never_reaches_one_second():
    stamp = stamp_from_seconds(10.9999999999)
    assert stamp.sec == 10
    assert stamp.nanosec < 1000000000


def test_zero():
    stamp = stamp_from_seconds(0.0)
    assert stamp.sec == 0
    assert stamp.nanosec == 0


def test_round_trip():
    stamp = stamp_from_seconds(1533451604.547540)
    t_s = stamp.sec + stamp.nanosec * 1e-9
    assert t_s == pytest.approx(1533451604.547540, abs=1e-6)


@pytest.fixture
def box():
    # w, l, h all different so that any swap of the size fields is caught.
    return GtBox(
        instance_token='dummy',
        tracking_name='car',
        translation=[1.0, 2.0, 3.0],
        size_wlh=[1.8, 4.5, 1.5],
        rotation_wxyz=[math.cos(0.15), 0.0, 0.0, math.sin(0.15)],  # yaw = 0.3
        yaw=0.3,
        velocity_xy=[0.0, 0.0],
        num_pts=10,
    )


@pytest.fixture
def header():
    return Header(stamp=stamp_from_seconds(1533151603.547590), frame_id=FRAME_ID)


def test_box_to_detection3d_size(box, header):
    detection = box_to_detection3d(box, header)
    assert detection.bbox.size.x == 4.5
    assert detection.bbox.size.y == 1.8
    assert detection.bbox.size.z == 1.5


def test_box_to_detection3d_orientation(box, header):
    detection = box_to_detection3d(box, header)
    assert detection.bbox.center.orientation.x == 0.0
    assert detection.bbox.center.orientation.y == 0.0
    assert detection.bbox.center.orientation.z == pytest.approx(math.sin(0.15))
    assert detection.bbox.center.orientation.w == pytest.approx(math.cos(0.15))


def test_box_to_detection3d_translation(box, header):
    detection = box_to_detection3d(box, header)
    assert detection.bbox.center.position.x == 1.0
    assert detection.bbox.center.position.y == 2.0
    assert detection.bbox.center.position.z == 3.0


def test_box_to_detection3d_results(box, header):
    detection = box_to_detection3d(box, header)
    assert len(detection.results) == 1
    assert detection.results[0].hypothesis.class_id == 'car'
    assert detection.results[0].hypothesis.score == 1.0
    assert detection.results[0].pose.pose == detection.bbox.center


def test_box_to_detection3d_header(box, header):
    detection = box_to_detection3d(box, header)
    assert detection.header.frame_id == 'map'
    assert detection.header.stamp.sec == 1533151603


def test_box_to_detection3d_id_does_not_leak_instance_token(box, header):
    detection = box_to_detection3d(box, header)
    # The GT instance token must never reach the tracker.
    assert detection.id == ''


def test_box_to_marker_scale(box, header):
    marker = box_to_marker(box, header, marker_id=42)
    assert marker.scale == Vector3(x=4.5, y=1.8, z=1.5)


def test_box_to_marker_color(box, header):
    marker = box_to_marker(box, header, marker_id=42)
    assert isinstance(marker.color, ColorRGBA)
    assert marker.color.r == 1.0
    assert marker.color.g == 1.0
    assert marker.color.b == 0.0
    assert marker.color.a > 0.0


@pytest.mark.parametrize('num_pts, alpha', [
    (0, 0.1),     # no points: faint, but still visible
    (10, 0.55),   # halfway
    (20, 1.0),    # saturation point
    (100, 1.0),   # capped, never above 1
])
def test_box_to_marker_alpha(box, header, num_pts, alpha):
    box.num_pts = num_pts
    marker = box_to_marker(box, header, marker_id=0)
    # ColorRGBA stores float32, hence the tolerance.
    assert marker.color.a == pytest.approx(alpha, abs=1e-6)


def test_box_to_marker_type_and_action(box, header):
    marker = box_to_marker(box, header, marker_id=42)
    assert marker.type == Marker.CUBE
    assert marker.action == Marker.ADD


def test_box_to_marker_header(box, header):
    marker = box_to_marker(box, header, marker_id=42)
    assert marker.header.frame_id == 'map'
    assert marker.header.stamp.sec == 1533151603


def test_box_to_marker_id(box, header):
    marker = box_to_marker(box, header, marker_id=42)
    assert marker.id == 42


def test_box_to_marker_pose(box, header):
    marker = box_to_marker(box, header, marker_id=42)
    assert marker.pose == box_pose(box)


@pytest.fixture
def keyframe(box):
    # Two boxes of different classes, so that the order of the outputs is checked.
    second = GtBox(
        instance_token='dummy-2',
        tracking_name='pedestrian',
        translation=[5.0, 6.0, 1.0],
        size_wlh=[0.6, 0.7, 1.8],
        rotation_wxyz=[1.0, 0.0, 0.0, 0.0],
        yaw=0.0,
        velocity_xy=[0.0, 0.0],
        num_pts=3,
    )
    return Keyframe(sample_token='s0', timestamp_s=1533151603.547590, boxes=[box, second])


def test_keyframe_to_msgs_counts(keyframe):
    detection_array, marker_array = keyframe_to_msgs(keyframe)
    assert len(detection_array.detections) == 2
    # One DELETEALL + one marker per box.
    assert len(marker_array.markers) == 3


def test_keyframe_to_msgs_deleteall_comes_first(keyframe):
    _, marker_array = keyframe_to_msgs(keyframe)
    assert marker_array.markers[0].action == Marker.DELETEALL
    assert all(m.action == Marker.ADD for m in marker_array.markers[1:])


def test_keyframe_to_msgs_marker_ids_unique(keyframe):
    _, marker_array = keyframe_to_msgs(keyframe)
    ids = [m.id for m in marker_array.markers[1:]]
    assert len(set(ids)) == len(ids)


def test_keyframe_to_msgs_keeps_box_order(keyframe):
    detection_array, marker_array = keyframe_to_msgs(keyframe)
    classes = [d.results[0].hypothesis.class_id for d in detection_array.detections]
    assert classes == ['car', 'pedestrian']
    assert marker_array.markers[2].pose == box_pose(keyframe.boxes[1])


def test_keyframe_to_msgs_same_header_everywhere(keyframe):
    detection_array, marker_array = keyframe_to_msgs(keyframe)
    expected = Header(stamp=stamp_from_seconds(keyframe.timestamp_s), frame_id=FRAME_ID)
    assert detection_array.header == expected
    assert all(d.header == expected for d in detection_array.detections)
    assert all(m.header == expected for m in marker_array.markers)


def test_keyframe_to_msgs_empty_keyframe():
    # A keyframe with no boxes must still clear the old markers in RViz.
    frame = Keyframe(sample_token='s0', timestamp_s=1.0, boxes=[])
    detection_array, marker_array = keyframe_to_msgs(frame)
    assert len(detection_array.detections) == 0
    assert len(marker_array.markers) == 1
    assert marker_array.markers[0].action == Marker.DELETEALL


@pytest.mark.parametrize('loop', [False, True])
def test_next_frame_index_middle(loop):
    assert next_frame_index(3, n_frames=40, loop=loop) == 4


def test_next_frame_index_last_without_loop_stops():
    assert next_frame_index(39, n_frames=40, loop=False) is None


def test_next_frame_index_last_with_loop_restarts():
    assert next_frame_index(39, n_frames=40, loop=True) == 0


def test_next_frame_index_single_frame():
    assert next_frame_index(0, n_frames=1, loop=False) is None
    assert next_frame_index(0, n_frames=1, loop=True) == 0


def test_next_frame_index_visits_every_frame_once():
    # Without loop, starting from 0, every keyframe is published exactly once.
    visited, index = [], 0
    while index is not None:
        visited.append(index)
        index = next_frame_index(index, n_frames=5, loop=False)
    assert visited == [0, 1, 2, 3, 4]
