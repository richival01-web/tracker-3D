"""Tests for scene_loader with a tiny fake dataset (no nuScenes data needed)."""

import numpy as np
import pytest

from nusc_tools.scene_loader import load_scene_keyframes, quat_from_yaw, yaw_from_quaternion


class FakeNuScenes:
    """Minimal stand-in exposing only what load_scene_keyframes uses."""

    version = 'fake'

    def __init__(self):
        self.scene = [{'name': 'scene-a', 'first_sample_token': 's0'}]
        self._tables = {
            'sample': {
                # Timestamps in microseconds, 0.5 s apart.
                's0': {'timestamp': 1_000_000, 'anns': ['a0', 'a1', 'a2'], 'next': 's1'},
                's1': {'timestamp': 1_500_000, 'anns': ['a3'], 'next': ''},
            },
            'sample_annotation': {
                'a0': self._ann('car-1', 'vehicle.car', 1.0, lidar=10),
                'a1': self._ann('cone-1', 'movable_object.trafficcone', 0.0, lidar=5),
                'a2': self._ann('ped-1', 'human.pedestrian.adult', 0.0, lidar=0),
                'a3': self._ann('car-1', 'vehicle.car', 1.0, lidar=8),
            },
        }

    @staticmethod
    def _ann(instance, category, yaw, lidar):
        return {'instance_token': instance, 'category_name': category,
                'translation': [1.0, 2.0, 0.5], 'size': [1.8, 4.5, 1.5],
                'rotation': quat_from_yaw(yaw),
                'num_lidar_pts': lidar, 'num_radar_pts': 0}

    def get(self, table, token):
        return self._tables[table][token]

    def box_velocity(self, ann_token):
        return np.array([2.0, 0.0, 0.0])


def test_yaw_from_quaternion():
    for yaw in [0.0, 0.3, -1.2, 3.0]:
        assert yaw_from_quaternion(quat_from_yaw(yaw)) == pytest.approx(yaw)


def test_load_scene_keyframes():
    frames = load_scene_keyframes(FakeNuScenes(), 'scene-a')

    # The linked list s0 -> s1 is followed to the end.
    assert [f.sample_token for f in frames] == ['s0', 's1']
    # Microseconds are converted to seconds.
    assert frames[1].timestamp_s - frames[0].timestamp_s == pytest.approx(0.5)
    # Traffic cone (not a tracking class) and the 0-point pedestrian are dropped.
    assert [b.tracking_name for b in frames[0].boxes] == ['car']
    box = frames[0].boxes[0]
    assert box.size_wlh == [1.8, 4.5, 1.5]
    assert box.yaw == pytest.approx(1.0)
    assert box.velocity_xy == [2.0, 0.0]


def test_keep_empty_boxes():
    frames = load_scene_keyframes(FakeNuScenes(), 'scene-a', drop_empty=False)
    assert [b.tracking_name for b in frames[0].boxes] == ['car', 'pedestrian']


def test_unknown_scene_raises():
    with pytest.raises(ValueError):
        load_scene_keyframes(FakeNuScenes(), 'scene-missing')
