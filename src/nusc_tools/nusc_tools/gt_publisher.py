"""ROS 2 node publishing simulated nuScenes detections as Detection3DArray + MarkerArray.

The detections are the GT boxes of a scene with configurable noise
(see nusc_tools.noise). With the default parameters there is no noise and
the GT is published as is (score aside).
"""

from dataclasses import fields
import os
from typing import List, Optional, Tuple

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import MarkerArray, Marker
from nuscenes.nuscenes import NuScenes

from nusc_tools.noise import NoiseConfig, SimDetection, frame_rng, simulate_detections
from nusc_tools.scene_loader import load_scene_keyframes


CLASS_COLORS = {
    'bicycle': (0.0, 1.0, 1.0),  # cyan
    'bus': (1.0, 0.0, 0.0),  # red
    'car': (1.0, 1.0, 0.0),  # yellow
    'motorcycle': (1.0, 0.0, 1.0),  # magenta
    'pedestrian': (0.0, 1.0, 0.0),  # green
    'trailer': (0.5, 0.5, 0.5),  # gray
    'truck': (0.0, 0.0, 1.0),  # blue
}
DEFAULT_COLOR = (1.0, 1.0, 1.0)  # white for unknown classes
FRAME_ID = 'map'


def stamp_from_seconds(t_s: float) -> Time:
    """Split a time in seconds into builtin_interfaces/Time (sec, nanosec).

    Assumes t_s >= 0.
    """
    sec = int(t_s)
    nanosec = int((t_s - sec) * 1e9)
    return Time(sec=sec, nanosec=nanosec)


def box_pose(box: SimDetection) -> Pose:
    """Return a geometry_msgs/Pose from a detection.

    The detection is in the global frame, and the Pose is published in the same frame.
    """
    pose = Pose()
    pose.position = Point(
        x=box.translation[0],
        y=box.translation[1],
        z=box.translation[2],
    )
    # nuScenes quaternion is [w, x, y, z]; the ROS message has named fields
    # x, y, z, w, so assign by name, never by position.
    pose.orientation = Quaternion(
        x=box.rotation_wxyz[1],
        y=box.rotation_wxyz[2],
        z=box.rotation_wxyz[3],
        w=box.rotation_wxyz[0],
    )
    return pose


def box_to_detection3d(box: SimDetection, header: Header) -> Detection3D:
    """Convert a simulated detection to a vision_msgs/Detection3D.

    The header (stamp of the keyframe + frame id) is built once per keyframe
    by the caller and shared by all its detections. The detection id stays
    empty: the GT instance token must never reach the tracker.
    """
    detection = Detection3D()
    detection.header = header

    detection.bbox.center = box_pose(box)
    # nuScenes size is wlh; BoundingBox3D.size is in the box frame,
    # with x along the heading: x = length, y = width, z = height.
    detection.bbox.size = Vector3(
        x=box.size_wlh[1],
        y=box.size_wlh[0],
        z=box.size_wlh[2],
    )

    # One hypothesis: class + simulated score, and the same pose as the box.
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = box.tracking_name
    hypothesis.hypothesis.score = box.score
    hypothesis.pose.pose = box_pose(box)
    detection.results.append(hypothesis)
    return detection


def box_to_marker(box: SimDetection, header: Header, marker_id: int) -> Marker:
    """Convert a simulated detection to a visualization_msgs/Marker.

    The header (stamp of the keyframe + frame id) is built once per keyframe
    by the caller and shared by all its detections. The marker_id is unique
    per detection in the keyframe, but can be reused in the next keyframe.
    """
    marker = Marker()
    marker.header = header
    marker.ns = 'detections'
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = box_pose(box)
    # nuScenes size is wlh; Marker.scale is in the box frame,
    # with x along the heading: x = length, y = width, z = height.
    marker.scale = Vector3(
        x=box.size_wlh[1],
        y=box.size_wlh[0],
        z=box.size_wlh[2],
    )
    # Color by tracking class; opacity by score, so false positives (lower
    # score on average) look fainter.
    r, g, b = CLASS_COLORS.get(box.tracking_name, DEFAULT_COLOR)
    a = 0.5 + 0.5 * box.score  # score in [0, 1] -> alpha in [0.5, 1.0]
    marker.color = ColorRGBA(r=r, g=g, b=b, a=a)
    return marker


def detections_to_msgs(timestamp_s: float, detections: List[SimDetection]
                       ) -> Tuple[Detection3DArray, MarkerArray]:
    """Convert the detections of one keyframe into the two messages published for it.

    One header (keyframe stamp + FRAME_ID) is shared by every message. The
    first marker is a DELETEALL: the number of detections changes between
    keyframes, so without it the markers with ids no longer used would stay
    in RViz forever. It must come first, otherwise it would also delete the
    detections of this keyframe.
    """
    header = Header(stamp=stamp_from_seconds(timestamp_s), frame_id=FRAME_ID)

    detection_array = Detection3DArray(header=header)
    marker_array = MarkerArray()
    marker_array.markers.append(Marker(header=header, action=Marker.DELETEALL))
    for i, det in enumerate(detections):
        detection_array.detections.append(box_to_detection3d(det, header))
        marker_array.markers.append(box_to_marker(det, header, marker_id=i))
    return detection_array, marker_array


def next_frame_index(index: int, n_frames: int, loop: bool) -> Optional[int]:
    """Return the index of the keyframe to publish after `index`, or None to stop.

    With `loop`, the last keyframe is followed directly by the first one, so
    no timer tick is wasted. Note: the stamps then jump back by the length of
    the scene (~20 s); a subscriber (the tracker, from G9) must detect the
    negative dt and reset, instead of predicting backwards in time.
    """
    if index + 1 < n_frames:
        return index + 1
    return 0 if loop else None


class GtPublisher(Node):
    """ROS 2 node publishing simulated detections, one keyframe per timer tick."""

    def __init__(self):
        super().__init__('gt_publisher')
        self.declare_parameter('scene', 'scene-0103')
        self.declare_parameter('version', 'v1.0-mini')
        self.declare_parameter('dataroot', os.environ.get('NUSC_ROOT',''))
        self.declare_parameter('period_s', 0.5)
        self.declare_parameter('loop', False)
        self.declare_parameter('seed', 0)
        # One ROS parameter per NoiseConfig field, with the same name and
        # default: the defaults live in one place only (the dataclass). The
        # defaults are floats, so values must be passed as floats too
        # (-p sigma_xy:=1.0, not 1).
        for f in fields(NoiseConfig):
            self.declare_parameter(f.name, f.default)
        self.scene = self.get_parameter('scene').get_parameter_value().string_value
        self.version = self.get_parameter('version').get_parameter_value().string_value
        self.dataroot = self.get_parameter('dataroot').get_parameter_value().string_value
        self.period_s = self.get_parameter('period_s').get_parameter_value().double_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        self.seed = self.get_parameter('seed').get_parameter_value().integer_value
        self.noise_cfg = NoiseConfig(
            **{f.name: self.get_parameter(f.name).value for f in fields(NoiseConfig)})
        if not self.dataroot:
            raise RuntimeError("dataroot is empty: set NUSC_ROOT or pass -p dataroot:=…")
        if self.seed < 0:
            # numpy refuses negative seeds; fail at startup, not at the first tick.
            raise RuntimeError(f"seed must be >= 0, got {self.seed}")
        self.dataset = NuScenes(version=self.version, dataroot=self.dataroot, verbose=False)
        self.keyframes = load_scene_keyframes(self.dataset, self.scene)  # load keyframes once
        if not self.keyframes:
            raise RuntimeError(f"Scene {self.scene} has no keyframes.")
        self.get_logger().info(
            f"GT Publisher initialized for scene {self.scene} in dataset {self.version}, "
            f"seed {self.seed}, {self.noise_cfg}.")
        # Index of the keyframe published at the next tick; None once the last
        # one has been published (the node then shuts down at the next tick).
        self.frame_index: Optional[int] = 0
        self.finished = False  # read by main() to leave the spin loop
        self.detection_pub = self.create_publisher(Detection3DArray, 'detections', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'detections/markers', 10)
        self.timer = self.create_timer(self.period_s, self.on_timer)


    def on_timer(self):
        """Publish the current keyframe, then move to the next one."""
        if self.frame_index is None:
            # The last keyframe went out one period ago: publish() only hands
            # the message to DDS, so destroying the node in the same tick could
            # drop it before the subscribers have received it.
            self.get_logger().info("All frames published. Shutting down.")
            self.finished = True
            return
        frame = self.keyframes[self.frame_index]
        # One generator per keyframe: the noise of frame k only depends on
        # (seed, k), so with `loop` every pass over the scene is identical.
        rng = frame_rng(self.seed, self.frame_index)
        detections = simulate_detections(frame.boxes, self.noise_cfg, rng)
        detection_array, marker_array = detections_to_msgs(frame.timestamp_s, detections)
        self.detection_pub.publish(detection_array)
        self.marker_pub.publish(marker_array)
        self.get_logger().info(
            f"Publishing frame {self.frame_index + 1}/{len(self.keyframes)}: "
            f"{len(frame.boxes)} GT boxes -> {len(detections)} detections.")

        self.frame_index = next_frame_index(self.frame_index, len(self.keyframes), self.loop)
        if self.frame_index == 0:
            self.get_logger().info("Looping back to the first frame.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GtPublisher()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=node.period_s)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
