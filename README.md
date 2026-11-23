# 3D Multi-Object Tracker in ROS 2

A tracking-by-detection 3D multi-object tracker for autonomous driving, built as a ROS 2 node in C++17 and evaluated on the nuScenes dataset with the official tracking metrics (AMOTA, AMOTP, ID switches).

## Planned features

- **Tracker core (C++17, Eigen):** a constant-velocity Kalman filter, Hungarian assignment (my own implementation) and track management (birth, confirmation, deletion), packaged as a library that does not depend on ROS
- **ROS 2 node:** subscribes to `vision_msgs/Detection3DArray` and publishes tracks with IDs plus RViz2 markers
- **Detection simulator (Python):** turns nuScenes ground-truth boxes into noisy detections (position/yaw noise, missed objects, false positives) with configurable parameters and a fixed seed
- **Evaluation:** results exported in the nuScenes tracking format and scored with the official devkit on `mini_val`
- **Robustness study:** how tracking performance changes as detections get worse
- **Tooling:** Docker (ROS 2 Jazzy), GoogleTest, GitHub Actions

## Pipeline

```
nuScenes v1.0-mini ──► detection simulator ──► tracker node ──► tracks + markers
   (devkit)              (Python, noise)        (C++ / ROS 2)        │
                                                                     ▼
                                                    nuScenes JSON ──► AMOTA / AMOTP
```

## Status

- [ ] Docker + colcon workspace
- [ ] nuScenes data loader and detection simulator
- [ ] Kalman filter, Hungarian assignment, track management (with tests)
- [ ] ROS 2 node and launch file
- [ ] Evaluation on `mini_val`
- [ ] Demo GIF and results table

## Data

This project uses [nuScenes](https://www.nuscenes.org/) v1.0-mini. **No dataset files are included in this repository.** You need to register on the nuScenes website and download the data yourself. nuScenes may be used for non-commercial purposes only; see the [nuScenes terms of use](https://www.nuscenes.org/terms-of-use).

## References

- H. Caesar et al., "nuScenes: A multimodal dataset for autonomous driving," CVPR 2020. [arXiv:1903.11027](https://arxiv.org/abs/1903.11027)
- X. Weng et al., "AB3DMOT: A Baseline for 3D Multi-Object Tracking and New Evaluation Metrics," [arXiv:1907.03961](https://arxiv.org/abs/1907.03961)
- Z. Pang et al., "SimpleTrack: Understanding and Rethinking 3D Multi-object Tracking," [arXiv:2111.09621](https://arxiv.org/abs/2111.09621)

## License

The code in this repository is released under the [MIT License](LICENSE). The license covers the code only; nuScenes data is subject to its own terms.
