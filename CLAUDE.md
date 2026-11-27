# CLAUDE.md – 3D Multi-Object Tracker in ROS 2

Instructions for Claude Code in this repository. Reply in Italian; code, comments, commit messages and README in English.

## Context

Personal project of Riccardo Valentini (M.Sc. Automotive Engineering, TUM), built for his portfolio and for applications to master's theses and working-student positions. It is a *tracking-by-detection* 3D tracker:

- Kalman filter, Hungarian assignment, track lifecycle
- tracker as a ROS 2 node in C++17, evaluated with the official nuScenes metrics (AMOTA, AMOTP, ID switches)
- in addition, a robustness study with degraded detections

Goals:

- **Public MVP by Sunday 6.12.2026** (tag v0.1)
- **v1.0 by 20.12.2026**

Pipeline:
1. nuScenes v1.0-mini read with `nuscenes-devkit`
2. Detection simulator (Python): ground-truth boxes + configurable noise
3. Tracker (C++): predict → association → update → track birth and death
4. Output: tracks with IDs in RViz2 + JSON in the nuScenes tracking format
5. Evaluation with the official devkit on `mini_val`

Out of scope: training a detector, cameras, the full dataset.

## How to work with Riccardo (important)

- **He must be able to explain every line of the repo** in an interview. Therefore:
  - explain the reasons behind your choices
  - prefer small steps he can review over large blocks generated in one go
  - after each non-trivial piece, summarise in 2–3 sentences what it does and which mistakes it avoids
- **The Kalman filter, the Hungarian algorithm and track management are the core of the project.** Guide him:
  - explain the algorithm
  - propose the signatures and the tests
  - let him write the implementation, then review it
  - write complete code only if he explicitly asks for it
- For Docker, CMake, ROS boilerplate, I/O and plotting scripts you can be more direct.
- At the start of a session:
  - look at the "Status" section at the bottom and at today's date
  - propose the task of the day, or how to catch up on days that fell behind
- At the end of a task:
  - check the "done when" criterion
  - update the "Status" section
  - propose a commit message
- Do not push or create tags without asking.

## Project rules (non-negotiable)

- **No code, parameters or data** from the TUM courses (Autonomous Systems, F1TENTH) or from the FTM chair / EDGAR.
- **No nuScenes data in the repo**, not even small extracts, example JSON files with real tokens, or images:
  - the dataset path comes from the environment variable `NUSC_ROOT` (mounted as a volume in Docker)
  - check `git status` before every commit
- nuScenes is for non-commercial use only: always cite the paper (Caesar et al., CVPR 2020).
- AB3DMOT (arXiv 1907.03961) and SimpleTrack (arXiv 2111.09621) are **references to read, not to copy**. Their ideas may be cited, but their code must not be pasted.
- MIT license for the code. Dependencies with a compatible license.
- Every change must pass `colcon test` before it is committed.

## Stack

- ROS 2 Jazzy in Docker (Ubuntu 24.04), colcon
- C++17, Eigen 3, GoogleTest; build with `ament_cmake`
- Python 3 with `nuscenes-devkit`, `numpy`, `pyquaternion`
- `vision_msgs/Detection3DArray` for detections and tracks; `visualization_msgs/MarkerArray` for visualisation
- RViz2 (in the container, via X11; chosen over Foxglove on 26.11); GitHub Actions from phase 2

## Planned repo layout

```
docker/Dockerfile
src/tracker_core/     # C++ library, NO ROS dependency in the core (only ament_cmake to build)
  include/tracker_core/  src/  test/
src/tracker_ros/      # (from G9) C++ ROS 2 node + launch files, depends on tracker_core
src/nusc_tools/       # Python: loader, detection simulator node, JSON export, evaluation
scripts/              # helper scripts (eval, plots, GIF)
docs/                 # pipeline diagram, GIF, plots
```

## Technical conventions (typical mistakes to avoid)

- **Frames:**
  - the tracker works in the nuScenes **global** frame
  - boxes from the devkit's `get_boxes` / `get_sample_data` may be in the sensor or ego frame: always check which frame you are in
- **Units:**
  - metres, radians, seconds
  - nuScenes timestamps are in **microseconds**: convert to seconds to compute `dt`
  - keyframes are at ~2 Hz (`dt` ≈ 0.5 s, but always use the real `dt`)
- **Dimensions:** the devkit uses the order **`wlh`** (width, length, height). The submission format uses `size = [w, l, h]`. Do not confuse it with `lwh`.
- **Rotations:**
  - nuScenes/pyquaternion quaternions are `[w, x, y, z]`, while `geometry_msgs/Quaternion` is `x, y, z, w`
  - yaw = rotation around z
  - in the Kalman filter, normalise the angle innovation to [-π, π]
  - consider correcting ~π flips, which are frequent in noisy detections
- **Filter state** (AB3DMOT style): `[x, y, z, yaw, l, w, h, vx, vy, vz]`, constant-velocity model. Q and R are parameters, not numbers hard-coded in the source.
- **nuScenes tracking classes:** `bicycle, bus, car, motorcycle, pedestrian, trailer, truck`. Only associate detections and tracks of the same class.
- **Reproducibility:** the simulator uses a fixed seed (ROS parameter); two runs with the same seed must be identical.
- **Results format** (`results.json`): `meta` (`use_camera`, `use_lidar`, `use_radar`, `use_map`, `use_external`) + `results` keyed by `sample_token`. For each track:
  - `sample_token`, `translation`, `size`, `rotation`
  - `velocity` (2D)
  - `tracking_id`, `tracking_name`, `tracking_score`
- **Splits:** `mini_train` (8 scenes) and `mini_val` (2 scenes). Evaluate on `mini_val`.

## Useful commands

```bash
# build the image (from the repo root; pass host UID/GID so files in the volume are yours)
docker build -f docker/Dockerfile -t tracker3d:dev \
  --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) docker/
# allow X11 windows from your own user (once per login session, on the host)
xhost +SI:localuser:$(id -un)
# start the container: repo mounted as colcon workspace, dataset read-only,
# X11 socket + /dev/dri (AMD iGPU via Mesa) for RViz2
docker run --rm -it --name tracker3d --hostname tracker3d \
  -v "$PWD":/ws -v "$NUSC_ROOT":/data/nuscenes:ro \
  -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --device /dev/dri \
  --group-add "$(getent group video | cut -d: -f3)" \
  --group-add "$(getent group render | cut -d: -f3)" \
  tracker3d:dev
# if RViz2 shows a black window / OpenGL errors: add -e LIBGL_ALWAYS_SOFTWARE=1
# build and test (inside the container)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && colcon test && colcon test-result --verbose
# official tracking evaluation (inside the container, NUSC_ROOT=/data/nuscenes)
python3 -m nuscenes.eval.tracking.evaluate --result_path results/results.json \
  --output_dir results/eval --eval_set mini_val --version v1.0-mini --dataroot $NUSC_ROOT
```
Notes:
- `ros2 pkg create` generates the package skeleton, but it must be reviewed: for example, it writes `find_package(eigen)` instead of `Eigen3` and does not set C++17.
- ROS Python nodes run with `/usr/bin/python3`: pip dependencies go into the container's system Python, not into a venv.
- `ament_python` packages need `extras_require={'test': ['pytest']}`, otherwise colcon uses `unittest` and fails when it finds no tests.

(Update this section once the launch files exist, from G9.)

## Daily schedule – Phase 1: MVP

| Day | Date | Task | Done when |
| --- | --- | --- | --- |
| G1 | Mon 23.11 | Public GitHub repo (MIT, provisional README, .gitignore); nuScenes registration and v1.0-mini download | Repo online, download started |
| G2 | Tue 24.11 | Dockerfile with ROS 2 Jazzy + Eigen + GoogleTest; colcon workspace with empty packages `tracker_core` (C++) and `nusc_tools` (Python) | `colcon build` passes in the container |
| G3 | Wed 25.11 | nuscenes-devkit tutorial; script that prints, for one scene, the annotated boxes of every keyframe in global coordinates with timestamps | Boxes of one scene printed with timestamps |
| G4 | Thu 26.11 | Python node that publishes the GT boxes as `Detection3DArray` + `MarkerArray` at keyframe rate | Boxes visible in RViz2 |
| G5 | Fri 27.11 | Noise configurable via ROS parameters (position and yaw σ, miss probability, false positives, score), fixed seed | Two runs with the same seed are identical |
| G6 | Sat 28.11 | Constant-velocity Kalman filter in C++/Eigen (AB3DMOT-style state), predict/update; GoogleTest on straight-line motion | Test green |
| G7 | Sun 29.11 | Association cost (centre distance) + Hungarian algorithm written by Riccardo, tests on small known cases | Tests green |
| G8 | Mon 30.11 | Track management: birth, confirmation (min hits), death (max age), per-class association; all in the ROS-free library | Test on a synthetic sequence with an object entering and leaving |
| G9 | Tue 01.12 | C++ ROS 2 node: subscribes to detections, publishes tracks and markers with IDs; single launch file for simulator + tracker | Pipeline started with a single `ros2 launch` |
| G10 | Wed 02.12 | End-to-end run on one scene; debugging of frames, timestamps and yaw | Stable IDs and visually consistent tracks |
| G11 | Thu 03.12 | Export of results as JSON in the nuScenes tracking format; demo GIF | Valid JSON + 10–20 s GIF |
| G12 | Fri 04.12 | Official evaluation with the devkit on `mini_val` (AMOTA, AMOTP, ID switches), without noise and with base noise | Table with the first numbers |
| G13 | Sat 05.12 | README: description, pipeline diagram, GIF, results table, one-command start in Docker, nuScenes credits and license | README complete |
| G14 | Sun 06.12 | Clean-up, rule check, tag v0.1 | Repo ready to publish |

## Schedule – Phase 2: polishing

| Day | Date | Task |
| --- | --- | --- |
| G15–G16 | 08–09.12 | 3D IoU/GIoU association cost; comparison with centre distance |
| G17–G18 | 10–11.12 | Parameter tuning (thresholds, min hits, max age, process noise) |
| G19–G21 | 12–14.12 | Sweep over 4–5 noise levels; AMOTA vs noise level plot |
| G22–G23 | 15–16.12 | GitHub Actions: build + test in the container |
| G24–G25 | 17–18.12 | Final README with the robustness plot; tag v1.0 |
| G26–G27 | 19–20.12 | Buffer for catching up |

## Status

Update this section at the end of each task (day, date, what is done, what is still open).

- 23.11.2026: G1 completed and closed. Public repo on GitHub (MIT, provisional README, .gitignore), commit + push of the .gitignore done (`main` in sync with `origin/main`). nuScenes v1.0-mini extracted to a folder outside the repo (`maps/`, `samples/`, `sweeps/`, `v1.0-mini/` directly inside it), with `NUSC_ROOT` pointing to it. Checked with nuscenes-devkit 1.2.0 (temporary venv, then removed): 10 scenes, 404 samples, mini_train 8/8, mini_val 2/2, no missing keyframe files. Note: Ubuntu 24.04 blocks system-wide `pip install` (PEP 668) → devkit in Docker.
- 23.11.2026: git history rewritten with the GitHub no-reply email (`239980864+richival01-web@users.noreply.github.com`, also in the repo's `git config user.email`) and force push done.
- 23.11.2026: G2 completed (one day early). `docker/Dockerfile`:
  - base `ros:jazzy-ros-base`
  - Eigen, GoogleTest, `ament_cmake_gtest`, `vision_msgs`, `visualization_msgs`, pytest
  - `nuscenes-devkit` 1.2.0 + pyquaternion via pip in the system Python, `numpy<2`
  - non-root user `dev` with the host's UID/GID

  Empty packages:
  - `tracker_core` (`ament_cmake`, C++17, Eigen3, no ROS dependency; folders `include/`, `src/`, `test/` with `.gitkeep`)
  - `nusc_tools` (`ament_python`, smoke test `test_import.py`)

  In the container `colcon build` passes for both packages; `colcon test`: 2 tests, 0 failures. Commit `147508d` pushed (`main` in sync with `origin/main`). No open items. Next: G3 (devkit tutorial + global boxes with timestamps), planned for Wed 25.11, but it can be brought forward to tomorrow.
- 24.11.2026: G3 completed (one day early).
  - `nusc_tools/scene_loader.py`: `load_scene_keyframes(nusc, scene)` follows the list of samples, converts timestamps from µs to s, reads the `sample_annotation` records (already in the global frame), keeps only the 7 tracking classes (the devkit's `category_to_tracking_name`) and discards boxes with 0 lidar+radar points, as the official evaluation does. Dataclasses `Keyframe` / `GtBox` (size `wlh`, quaternion `wxyz`, yaw, 2D velocity, `instance_token`). To be reused in G4.
  - `nusc_tools/print_scene.py` (console script `print_scene`): `ros2 run nusc_tools print_scene --scene scene-0103 --max-frames 3`.
  - `test/test_scene_loader.py`: 4 tests with a fake dataset (no nuScenes data). `colcon test`: 6 tests, 0 failures.
  - Dockerfile: added `motmetrics==1.4.0` and `pandas<2.3`, needed by `nuscenes.eval.tracking`. Devkit 1.2.0 requires 1.4.0 on Python ≥ 3.10; 1.1.3 breaks on 3.12. Image rebuilt and smoke test of the devkit's MOT accumulator ok.
  - Observations: mini_val = scene-0103 (40 keyframes, 19.4 s) and scene-0916 (41 keyframes, 19.9 s), `dt` ≈ 0.500 s. Some objects disappear for one keyframe (0 points) and then reappear, which matters for `max_age` in G8.
  - Commit `dd0cc7d` pushed. Next: G4 (GT publisher as `Detection3DArray` + `MarkerArray`).
- 26.11.2026: RViz2 added to the dev image (commit `1cd1bf5`), chosen over Foxglove. CLAUDE.md added to the repo in English.
- 27.11.2026: G4 completed. `nusc_tools/gt_publisher.py` (console script `gt_publisher`): publishes one keyframe per tick (`period_s`, default 0.5 s) on `detections` (`Detection3DArray`) and `detections/markers` (`MarkerArray`, DELETEALL first), frame `map`, parameters `scene`, `version`, `dataroot` (default `$NUSC_ROOT`), `loop`. With `loop:=false` the node exits by itself one tick after the last keyframe (so DDS can still deliver it; checked with a fake 5-frame scene: 5/5 received, exit code 0). Pure helpers `keyframe_to_msgs` and `next_frame_index` tested in `test/test_gt_publisher.py` (39 tests in `nusc_tools`, all green). Boxes checked in RViz2; view saved in `rviz/gt.rviz` (Fixed Frame `map`, MarkerArray on `/detections/markers`). Commits `5971365` + `bb6c72b` pushed. Next: G5 (noise model).
  - **Note for G9:** with `loop:=true` the stamps jump back by ~20 s when the scene restarts. The tracker node must detect a negative `dt` and reset its tracks instead of predicting backwards.
- 27.11.2026: G5 completed.
  - `nusc_tools/noise.py`: `NoiseConfig` (sigma_xy, sigma_yaw, p_flip, p_miss, fp_rate, TP/FP score mean, score sigma; all off by default), `SimDetection`, `frame_rng(seed, frame_index)` (one numpy Generator per keyframe), `simulate_detections`. All random numbers of a box are drawn in a fixed order before deciding a miss/flip ("common random numbers": a sweep over p_miss does not move the surviving boxes). FPs: Poisson(fp_rate) per keyframe, uniform in the GT bounding rectangle + `FP_MARGIN_M` (10 m), class/size/z copied from a random GT box. Output shuffled. `wrap_angle` (→ [-π, π)) added to `scene_loader.py`.
  - `gt_publisher.py`: one ROS parameter per `NoiseConfig` field (same name and default) + `seed` (must be >= 0); publishes the simulated score; `keyframe_to_msgs` became `detections_to_msgs(timestamp_s, detections)`; marker opacity from the score.
  - Tests: `test/test_noise.py` new, `test_gt_publisher.py` adapted (synthetic timestamps instead of values from scene-0103). 70 tests, all green (pytest in the container). Same seed → identical `/detections` over two runs of the node (fake 6-frame scene), other seed → different.
  - `colcon test` green, noisy detections checked in RViz2 on scene-0103. Commit `66fd77a` pushed. Next: G6 (Kalman filter in C++/Eigen). Small extra before G6: publish the GT markers on a separate topic for a visual GT vs noisy comparison.
