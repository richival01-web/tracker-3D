"""Tests for the noise model (pure Python, no ROS and no nuScenes data)."""

import copy
import math

import numpy as np
import pytest

from nusc_tools.noise import (
    FP_MARGIN_M, NoiseConfig, _perturb, _sample_false_positive, _sample_score,
    frame_rng, simulate_detections)
from nusc_tools.scene_loader import GtBox, quat_from_yaw, wrap_angle, yaw_from_quaternion


def make_box(name, x, y, size_wlh, yaw, z=1.0):
    return GtBox(
        instance_token=f'tok-{name}-{x}-{y}',
        tracking_name=name,
        translation=[x, y, z],
        size_wlh=size_wlh,
        rotation_wxyz=quat_from_yaw(yaw),
        yaw=yaw,
        velocity_xy=[0.0, 0.0],
        num_pts=10,
    )


@pytest.fixture
def boxes():
    # Different classes, sizes and positions, so that copies can be traced back.
    return [
        make_box('car', 100.0, 200.0, [1.8, 4.5, 1.5], 0.3),
        make_box('pedestrian', 110.0, 230.0, [0.6, 0.7, 1.8], -2.0),
        make_box('truck', 90.0, 215.0, [2.5, 8.0, 3.2], 3.0),
    ]


NOISY = NoiseConfig(sigma_xy=0.5, sigma_yaw=0.1, p_flip=0.2, p_miss=0.3, fp_rate=2.0)


def key(det):
    """Comparable summary of a detection (GtBox or SimDetection)."""
    return (det.tracking_name, tuple(det.translation), tuple(det.size_wlh), round(det.yaw, 9))


# --- wrap_angle ------------------------------------------------------------

@pytest.mark.parametrize('angle, expected', [
    (0.0, 0.0),
    (math.pi / 2, math.pi / 2),
    (math.pi, -math.pi),          # the interval is [-pi, pi): pi maps to -pi
    (-math.pi, -math.pi),
    (3 * math.pi, -math.pi),
    (-3 * math.pi / 2, math.pi / 2),
    (2 * math.pi + 0.1, 0.1),
])
def test_wrap_angle_values(angle, expected):
    assert wrap_angle(angle) == pytest.approx(expected, abs=1e-12)


def test_wrap_angle_range():
    for angle in np.linspace(-20.0, 20.0, 10001):
        assert -math.pi <= wrap_angle(angle) < math.pi


# --- frame_rng -------------------------------------------------------------

def test_frame_rng_same_seed_same_sequence():
    assert np.array_equal(frame_rng(42, 3).random(5), frame_rng(42, 3).random(5))


def test_frame_rng_other_seed_or_frame_differs():
    reference = frame_rng(42, 3).random(5)
    assert not np.array_equal(reference, frame_rng(43, 3).random(5))
    assert not np.array_equal(reference, frame_rng(42, 4).random(5))


def test_frame_rng_frames_are_independent(boxes):
    # Frame 5 gets the same noise whether or not frames 0-4 were generated before.
    alone = simulate_detections(boxes, NOISY, frame_rng(7, 5))
    for k in range(5):
        simulate_detections(boxes, NOISY, frame_rng(7, k))
    after = simulate_detections(boxes, NOISY, frame_rng(7, 5))
    assert alone == after


# --- simulate_detections ---------------------------------------------------

def test_zero_noise_returns_the_gt(boxes):
    detections = simulate_detections(boxes, NoiseConfig(), frame_rng(0, 0))
    # Compared as sets: the output is shuffled.
    assert sorted(map(key, detections)) == sorted(map(key, boxes))


def test_same_seed_identical_output(boxes):
    # The "done when" of G5, on the pure function.
    first = simulate_detections(boxes, NOISY, frame_rng(42, 7))
    second = simulate_detections(boxes, NOISY, frame_rng(42, 7))
    assert first == second


def test_other_seed_different_output(boxes):
    first = simulate_detections(boxes, NOISY, frame_rng(42, 7))
    other = simulate_detections(boxes, NOISY, frame_rng(43, 7))
    assert first != other


def test_input_boxes_not_modified(boxes):
    before = copy.deepcopy(boxes)
    detections = simulate_detections(boxes, NOISY, frame_rng(1, 1))
    assert boxes == before
    # No list shared with the GT: modifying a detection cannot change the GT.
    gt_lists = [id(b.size_wlh) for b in boxes] + [id(b.translation) for b in boxes]
    for det in detections:
        assert id(det.size_wlh) not in gt_lists
        assert id(det.translation) not in gt_lists


def test_p_miss_one_drops_every_box(boxes):
    detections = simulate_detections(boxes, NoiseConfig(p_miss=1.0), frame_rng(0, 0))
    assert detections == []


def test_p_miss_zero_keeps_every_box(boxes):
    detections = simulate_detections(boxes, NoiseConfig(p_miss=0.0, sigma_xy=1.0),
                                     frame_rng(0, 0))
    assert len(detections) == len(boxes)


def test_p_miss_rate():
    box = make_box('car', 0.0, 0.0, [1.8, 4.5, 1.5], 0.0)
    kept = sum(_perturb(box, NoiseConfig(p_miss=0.3), frame_rng(3, k)) is not None
               for k in range(10000))
    assert kept / 10000 == pytest.approx(0.7, abs=0.02)


def test_empty_frame_has_no_false_positives():
    assert simulate_detections([], NoiseConfig(fp_rate=5.0), frame_rng(0, 0)) == []


def test_false_positive_count_is_poisson(boxes):
    cfg = NoiseConfig(fp_rate=2.0)  # no miss: every extra detection is a FP
    counts = [len(simulate_detections(boxes, cfg, frame_rng(5, k))) - len(boxes)
              for k in range(5000)]
    # For a Poisson distribution mean and variance are both equal to the rate.
    assert np.mean(counts) == pytest.approx(2.0, abs=0.1)
    assert np.var(counts) == pytest.approx(2.0, abs=0.2)


def test_output_is_shuffled(boxes):
    # With zero noise the GT order is lost on some frames: the tracker
    # cannot use the order of the detections as a hint of their identity.
    many = [make_box('car', float(i), 0.0, [1.8, 4.5, 1.5], 0.0) for i in range(10)]
    orders = {tuple(d.translation[0] for d in simulate_detections(many, NoiseConfig(),
                                                                  frame_rng(0, k)))
              for k in range(20)}
    assert len(orders) > 1


@pytest.mark.parametrize('cfg', [NOISY, NoiseConfig(sigma_yaw=1.0, p_flip=0.5, fp_rate=3.0)])
def test_angles_and_scores_valid(boxes, cfg):
    for k in range(500):
        for det in simulate_detections(boxes, cfg, frame_rng(11, k)):
            assert -math.pi <= det.yaw < math.pi
            # The quaternion and the yaw field describe the same rotation.
            diff = wrap_angle(yaw_from_quaternion(det.rotation_wxyz) - det.yaw)
            assert abs(diff) < 1e-9
            assert 0.0 <= det.score <= 1.0


# --- _perturb --------------------------------------------------------------

def test_position_noise_std():
    box = make_box('car', 100.0, 200.0, [1.8, 4.5, 1.5], 0.3)
    cfg = NoiseConfig(sigma_xy=0.5)
    dets = [_perturb(box, cfg, frame_rng(9, k)) for k in range(10000)]
    dx = np.array([d.translation[0] - 100.0 for d in dets])
    dy = np.array([d.translation[1] - 200.0 for d in dets])
    assert np.std(dx) == pytest.approx(0.5, rel=0.05)
    assert np.std(dy) == pytest.approx(0.5, rel=0.05)
    assert abs(np.mean(dx)) < 0.02
    # z and size are not perturbed.
    assert all(d.translation[2] == 1.0 and d.size_wlh == [1.8, 4.5, 1.5] for d in dets)


def test_flip_changes_only_the_heading():
    box = make_box('car', 100.0, 200.0, [1.8, 4.5, 1.5], 0.3)
    det = _perturb(box, NoiseConfig(p_flip=1.0), frame_rng(0, 0))
    assert det.translation == box.translation
    assert det.yaw == pytest.approx(wrap_angle(0.3 + math.pi))


def test_flip_rate():
    box = make_box('car', 0.0, 0.0, [1.8, 4.5, 1.5], 0.3)
    flipped = sum(abs(_perturb(box, NoiseConfig(p_flip=0.2), frame_rng(4, k)).yaw - 0.3) > 1.0
                  for k in range(10000))
    assert flipped / 10000 == pytest.approx(0.2, abs=0.02)


def test_common_random_numbers():
    # Changing p_miss or p_flip must not change the position noise of the
    # boxes that survive (same seed): the sweep varies one factor at a time.
    box = make_box('car', 100.0, 200.0, [1.8, 4.5, 1.5], 0.3)
    for k in range(200):
        base = _perturb(box, NoiseConfig(sigma_xy=0.5), frame_rng(2, k))
        other = _perturb(box, NoiseConfig(sigma_xy=0.5, p_miss=0.5, p_flip=0.5), frame_rng(2, k))
        if other is not None:
            assert other.translation == base.translation
            assert other.score == base.score


# --- _sample_false_positive ------------------------------------------------

def test_false_positives_inside_enlarged_rectangle(boxes):
    x_min, x_max = 90.0 - FP_MARGIN_M, 110.0 + FP_MARGIN_M
    y_min, y_max = 200.0 - FP_MARGIN_M, 230.0 + FP_MARGIN_M
    for k in range(1000):
        fp = _sample_false_positive(boxes, NoiseConfig(), frame_rng(6, k))
        assert x_min <= fp.translation[0] <= x_max
        assert y_min <= fp.translation[1] <= y_max


def test_false_positive_single_box_square():
    box = make_box('car', 100.0, 200.0, [1.8, 4.5, 1.5], 0.3)
    for k in range(500):
        fp = _sample_false_positive([box], NoiseConfig(), frame_rng(6, k))
        assert abs(fp.translation[0] - 100.0) <= FP_MARGIN_M
        assert abs(fp.translation[1] - 200.0) <= FP_MARGIN_M


def test_false_positive_copies_class_size_and_z(boxes):
    templates = {(b.tracking_name, tuple(b.size_wlh), b.translation[2]) for b in boxes}
    seen = set()
    for k in range(300):
        fp = _sample_false_positive(boxes, NoiseConfig(), frame_rng(8, k))
        signature = (fp.tracking_name, tuple(fp.size_wlh), fp.translation[2])
        assert signature in templates
        assert all(fp.size_wlh is not b.size_wlh for b in boxes)
        seen.add(signature)
    assert seen == templates  # every GT box gets used as a template


def test_false_positive_scores_lower_than_true_positives(boxes):
    cfg = NoiseConfig()
    fp_scores = [_sample_false_positive(boxes, cfg, frame_rng(10, k)).score for k in range(2000)]
    tp_scores = [_perturb(boxes[0], cfg, frame_rng(10, k)).score for k in range(2000)]
    assert np.mean(fp_scores) == pytest.approx(cfg.score_fp_mean, abs=0.01)
    assert np.mean(tp_scores) == pytest.approx(cfg.score_tp_mean, abs=0.01)


# --- _sample_score ---------------------------------------------------------

def test_score_is_clipped_and_python_float():
    rng = frame_rng(0, 0)
    scores = [_sample_score(0.95, 0.5, rng) for _ in range(1000)]  # often above 1
    assert all(0.0 <= s <= 1.0 for s in scores)
    assert max(scores) == 1.0 and min(scores) == 0.0
    assert all(type(s) is float for s in scores)
