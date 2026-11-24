"""Load the ground-truth boxes of a nuScenes scene, keyframe by keyframe.

All boxes are returned in the nuScenes GLOBAL frame. The `sample_annotation`
records are stored in the global frame already, so no transformation is
applied here. (Note: `NuScenes.get_sample_data` / `get_boxes` would instead
return boxes in the sensor frame.)

Conventions kept from the devkit:
  - size is `wlh` (width, length, height), in meters
  - rotation is a quaternion `[w, x, y, z]`
  - timestamps are converted from microseconds to seconds here
"""

from dataclasses import dataclass, field
from typing import List, Optional

from pyquaternion import Quaternion

# Same class mapping as the official tracking evaluation (needs motmetrics).
from nuscenes.eval.tracking.utils import category_to_tracking_name

US_PER_S = 1e6


@dataclass
class GtBox:
    """One annotated object at one keyframe, in the global frame."""

    instance_token: str        # identity of the object across keyframes
    tracking_name: str         # one of the 7 nuScenes tracking classes
    translation: List[float]   # box center [x, y, z] in meters
    size_wlh: List[float]      # [width, length, height] in meters
    rotation_wxyz: List[float]  # quaternion [w, x, y, z]
    yaw: float                 # rotation around z in radians, in [-pi, pi]
    velocity_xy: List[float]   # [vx, vy] in m/s; NaN if not estimable
    num_pts: int               # lidar + radar points inside the box


@dataclass
class Keyframe:
    """One annotated sample (~2 Hz) of a scene."""

    sample_token: str
    timestamp_s: float
    boxes: List[GtBox] = field(default_factory=list)


def yaw_from_quaternion(q_wxyz: List[float]) -> float:
    """Return the rotation around z (yaw) of a [w, x, y, z] quaternion."""
    return Quaternion(q_wxyz).yaw_pitch_roll[0]


def find_scene(nusc, scene_name: str) -> dict:
    """Return the scene record with the given name (e.g. 'scene-0103')."""
    for scene in nusc.scene:
        if scene['name'] == scene_name:
            return scene
    raise ValueError(f'Scene {scene_name!r} not found in {nusc.version}')


def load_scene_keyframes(nusc, scene_name: str,
                         drop_empty: bool = True) -> List[Keyframe]:
    """Return all keyframes of a scene with their GT boxes in the global frame.

    Only objects of the 7 tracking classes are kept. If `drop_empty` is True,
    boxes with no lidar and no radar points are dropped, as the official
    evaluation does.
    """
    scene = find_scene(nusc, scene_name)
    keyframes = []

    # Samples form a linked list: first_sample_token -> next -> ... -> ''.
    sample_token: Optional[str] = scene['first_sample_token']
    while sample_token:
        sample = nusc.get('sample', sample_token)
        frame = Keyframe(sample_token=sample_token,
                         timestamp_s=sample['timestamp'] / US_PER_S)

        for ann_token in sample['anns']:
            ann = nusc.get('sample_annotation', ann_token)
            tracking_name = category_to_tracking_name(ann['category_name'])
            if tracking_name is None:  # e.g. barrier, traffic cone
                continue
            num_pts = ann['num_lidar_pts'] + ann['num_radar_pts']
            if drop_empty and num_pts == 0:
                continue

            # Finite differences with the prev/next annotation of the same
            # instance; NaN if the object appears in a single keyframe.
            velocity = nusc.box_velocity(ann_token)

            frame.boxes.append(GtBox(
                instance_token=ann['instance_token'],
                tracking_name=tracking_name,
                translation=list(ann['translation']),
                size_wlh=list(ann['size']),
                rotation_wxyz=list(ann['rotation']),
                yaw=yaw_from_quaternion(ann['rotation']),
                velocity_xy=[float(velocity[0]), float(velocity[1])],
                num_pts=num_pts,
            ))

        keyframes.append(frame)
        sample_token = sample['next']

    return keyframes
