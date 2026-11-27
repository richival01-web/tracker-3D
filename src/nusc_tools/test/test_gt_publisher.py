"""Tests for gt_publisher."""

import math

from geometry_msgs.msg import Vector3
import pytest
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

from nusc_tools.gt_publisher import (
    FRAME_ID, box_pose, box_to_detection3d, box_to_marker, detections_to_msgs,
    next_frame_index, stamp_from_seconds)
from nusc_tools.noise import SimDetection

# Synthetic timestamp with the magnitude of the nuScenes ones (~1.5e9 s);
# no value is taken from the dataset.
TIMESTAMP_S = 1500000000.547590


def test_nuscenes_like_timestamp():
    stamp = stamp_from_seconds(TIMESTAMP_S)
    assert stamp.sec == 1500000000
    assert abs(stamp.nanosec - 547590000) < 1000


def test_nuscenes_like_timestamp_small_fraction():
    stamp = stamp_from_seconds(1500000001.048025)
    assert stamp.sec == 1500000001
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
    stamp = stamp_from_seconds(1500000002.547540)
    t_s = stamp.sec + stamp.nanosec * 1e-9
    assert t_s == pytest.approx(1500000002.547540, abs=1e-6)


@pytest.fixture
def box():
    # w, l, h all different so that any swap of the size fields is caught.
    return SimDetection(
        tracking_name='car',
        translation=[1.0, 2.0, 3.0],
        size_wlh=[1.8, 4.5, 1.5],
        rotation_wxyz=[math.cos(0.15), 0.0, 0.0, math.sin(0.15)],  # yaw = 0.3
        yaw=0.3,
        score=0.7,
    )


@pytest.fixture
def header():
    return Header(stamp=stamp_from_seconds(TIMESTAMP_S), frame_id=FRAME_ID)


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
    # The simulated score is published, not a fixed 1.0.
    assert detection.results[0].hypothesis.score == pytest.approx(0.7)
    assert detection.results[0].pose.pose == detection.bbox.center


def test_box_to_detection3d_header(box, header):
    detection = box_to_detection3d(box, header)
    assert detection.header.frame_id == 'map'
    assert detection.header.stamp.sec == 1500000000


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


@pytest.mark.parametrize('score, alpha', [
    (0.0, 0.5),   # lowest score: faint, but still visible
    (0.5, 0.75),  # halfway
    (1.0, 1.0),   # highest score: opaque
])
def test_box_to_marker_alpha(box, header, score, alpha):
    box.score = score
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
    assert marker.header.stamp.sec == 1500000000


def test_box_to_marker_id(box, header):
    marker = box_to_marker(box, header, marker_id=42)
    assert marker.id == 42


def test_box_to_marker_pose(box, header):
    marker = box_to_marker(box, header, marker_id=42)
    assert marker.pose == box_pose(box)


@pytest.fixture
def detections(box):
    # Two detections of different classes, so that the order of the outputs is checked.
    second = SimDetection(
        tracking_name='pedestrian',
        translation=[5.0, 6.0, 1.0],
        size_wlh=[0.6, 0.7, 1.8],
        rotation_wxyz=[1.0, 0.0, 0.0, 0.0],
        yaw=0.0,
        score=0.4,
    )
    return [box, second]


def test_detections_to_msgs_counts(detections):
    detection_array, marker_array = detections_to_msgs(TIMESTAMP_S, detections)
    assert len(detection_array.detections) == 2
    # One DELETEALL + one marker per detection.
    assert len(marker_array.markers) == 3


def test_detections_to_msgs_deleteall_comes_first(detections):
    _, marker_array = detections_to_msgs(TIMESTAMP_S, detections)
    assert marker_array.markers[0].action == Marker.DELETEALL
    assert all(m.action == Marker.ADD for m in marker_array.markers[1:])


def test_detections_to_msgs_marker_ids_unique(detections):
    _, marker_array = detections_to_msgs(TIMESTAMP_S, detections)
    ids = [m.id for m in marker_array.markers[1:]]
    assert len(set(ids)) == len(ids)


def test_detections_to_msgs_keeps_order(detections):
    detection_array, marker_array = detections_to_msgs(TIMESTAMP_S, detections)
    classes = [d.results[0].hypothesis.class_id for d in detection_array.detections]
    assert classes == ['car', 'pedestrian']
    assert marker_array.markers[2].pose == box_pose(detections[1])


def test_detections_to_msgs_same_header_everywhere(detections):
    detection_array, marker_array = detections_to_msgs(TIMESTAMP_S, detections)
    expected = Header(stamp=stamp_from_seconds(TIMESTAMP_S), frame_id=FRAME_ID)
    assert detection_array.header == expected
    assert all(d.header == expected for d in detection_array.detections)
    assert all(m.header == expected for m in marker_array.markers)


def test_detections_to_msgs_empty_keyframe():
    # A keyframe with no detections must still clear the old markers in RViz.
    detection_array, marker_array = detections_to_msgs(1.0, [])
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
