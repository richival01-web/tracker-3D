"""Turn the GT boxes of a keyframe into noisy, detector-like detections.

Effects (all off by default, so that NoiseConfig() returns the GT):
  - Gaussian noise on the x, y position and on the yaw
  - heading flips (yaw + pi), frequent in real detectors
  - missed detections
  - false positives, Poisson-distributed per keyframe
  - confidence scores, lower on average for false positives

Reproducibility: every random draw goes through the numpy Generator passed
in, never through `random` or the global `np.random`. Use one generator per
keyframe (`frame_rng`), so that the noise of keyframe k only depends on
(seed, k).
"""

from dataclasses import dataclass
import math
from typing import List, Optional

import numpy as np

from nusc_tools.scene_loader import GtBox, quat_from_yaw, wrap_angle

FP_MARGIN_M = 10.0  # false positives are sampled up to this far outside the GT boxes


@dataclass(frozen=True)
class NoiseConfig:
    """Parameters of the noise model (units: m, rad, probabilities in [0, 1])."""

    sigma_xy: float = 0.0       # std dev of the x and y position error, m
    sigma_yaw: float = 0.0      # std dev of the yaw error, rad
    p_flip: float = 0.0         # probability of a heading flip (yaw + pi)
    p_miss: float = 0.0         # probability of missing a GT box
    fp_rate: float = 0.0        # mean number of false positives per keyframe
    score_tp_mean: float = 0.8  # mean score of true positives
    score_fp_mean: float = 0.4  # mean score of false positives
    score_sigma: float = 0.1    # std dev of the score, for TPs and FPs


@dataclass
class SimDetection:
    """One simulated detection, in the global frame.

    It has no instance token on purpose: the tracker must never see the GT
    identity.
    """

    tracking_name: str
    translation: List[float]    # box center [x, y, z] in meters
    size_wlh: List[float]       # [width, length, height] in meters
    rotation_wxyz: List[float]  # quaternion [w, x, y, z], pure rotation around z
    yaw: float                  # in [-pi, pi)
    score: float                # in [0, 1]


def frame_rng(seed: int, frame_index: int) -> np.random.Generator:
    """Return the random generator of one keyframe.

    Seeding with the pair (seed, frame_index) makes each keyframe
    reproducible on its own: the noise of keyframe k does not depend on how
    many numbers the previous keyframes consumed, and with `loop` every pass
    over the scene is identical. Both values must be >= 0.
    """
    return np.random.default_rng([seed, frame_index])


def _sample_score(mean: float, sigma: float, rng: np.random.Generator) -> float:
    """Return a Gaussian score clipped to [0, 1]."""
    # float(): np.clip returns np.float64, ROS messages want a Python float.
    return float(np.clip(rng.normal(mean, sigma), 0.0, 1.0))


def _perturb(box: GtBox, cfg: NoiseConfig,
             rng: np.random.Generator) -> Optional[SimDetection]:
    """Return a noisy copy of a GT box, or None if the box is missed.

    All the random numbers are drawn first, in a fixed order, whatever the
    outcome. So the noise of the boxes that survive does not depend on
    p_miss or p_flip: with the same seed, a sweep over p_miss changes only
    which boxes are missed, not where the others are ("common random
    numbers").
    """
    u_miss = rng.random()
    dx, dy = rng.normal(0.0, cfg.sigma_xy, size=2)
    dyaw = rng.normal(0.0, cfg.sigma_yaw)
    u_flip = rng.random()
    score = _sample_score(cfg.score_tp_mean, cfg.score_sigma, rng)

    if u_miss < cfg.p_miss:
        return None

    yaw = box.yaw + dyaw
    if u_flip < cfg.p_flip:
        yaw += math.pi  # the box stays where it is: only its heading is wrong
    yaw = wrap_angle(yaw)

    return SimDetection(
        tracking_name=box.tracking_name,
        translation=[box.translation[0] + float(dx),
                     box.translation[1] + float(dy),
                     box.translation[2]],
        size_wlh=list(box.size_wlh),  # copy: never share a list with the GT
        # Rebuilt from the yaw: pitch and roll are dropped, which is fine
        # because nuScenes boxes are (almost) pure rotations around z.
        rotation_wxyz=quat_from_yaw(yaw),
        yaw=yaw,
        score=score,
    )


def _sample_false_positive(boxes: List[GtBox], cfg: NoiseConfig,
                           rng: np.random.Generator) -> SimDetection:
    """Return one false positive placed near the GT boxes of the keyframe.

    Class and size are copied from a random GT box, so they are plausible
    together (no 6 m pedestrians). The position is uniform in the rectangle
    that contains all GT boxes, enlarged by FP_MARGIN_M on each side, so the
    false positives land where the tracker has tracks. Assumes `boxes` is
    not empty.
    """
    template = boxes[rng.integers(len(boxes))]

    xs = [b.translation[0] for b in boxes]
    ys = [b.translation[1] for b in boxes]
    x = rng.uniform(min(xs) - FP_MARGIN_M, max(xs) + FP_MARGIN_M)
    y = rng.uniform(min(ys) - FP_MARGIN_M, max(ys) + FP_MARGIN_M)

    # A false positive has no true heading: uniform in [-pi, pi).
    yaw = rng.uniform(-math.pi, math.pi)

    return SimDetection(
        tracking_name=template.tracking_name,
        # z of the template: about the ground height of the scene.
        translation=[float(x), float(y), template.translation[2]],
        size_wlh=list(template.size_wlh),
        rotation_wxyz=quat_from_yaw(yaw),
        yaw=float(yaw),
        score=_sample_score(cfg.score_fp_mean, cfg.score_sigma, rng),
    )


def simulate_detections(boxes: List[GtBox], cfg: NoiseConfig,
                        rng: np.random.Generator) -> List[SimDetection]:
    """Return the simulated detections of one keyframe, in random order.

    The input boxes are never modified. The output is shuffled because a
    real detector gives no ordering guarantee: the tracker must not be able
    to use the GT order of the boxes as a hint of their identity.
    """
    detections = []
    for box in boxes:
        detection = _perturb(box, cfg, rng)
        if detection is not None:
            detections.append(detection)

    # No GT box -> no rectangle to place false positives in.
    n_fp = int(rng.poisson(cfg.fp_rate)) if boxes else 0
    for _ in range(n_fp):
        detections.append(_sample_false_positive(boxes, cfg, rng))

    order = rng.permutation(len(detections))
    return [detections[i] for i in order]
