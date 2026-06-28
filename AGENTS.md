# FAST-LIVO2 Agent Guide

## Project Overview

FAST-LIVO2 is a **ROS Noetic** (Ubuntu 20.04) catkin package (`fast_livo`) implementing real-time LiDAR-Inertial-Visual odometry. Written in **C++17**, built with **CMake + catkin_make**, containerized via Podman/Docker.

**Developers**: Chunran Zheng <zhengcr@connect.hku.hk>, Prof. Fu Zhang <fuzhang@hku.hk>

## Repository Structure

```
FAST-LIVO2/
├── include/                    # Headers (.h)
│   ├── LIVMapper.h             # Main orchestrator class
│   ├── IMU_Processing.h        # IMU preintegration & undistortion
│   ├── preprocess.h            # LiDAR point cloud preprocessing
│   ├── voxel_map.h             # Voxel octree map (LIO backend)
│   ├── vio.h                   # Visual-Inertial Odometry manager
│   ├── frame.h                 # Camera frame with features
│   ├── visual_point.h          # 3D visual map point
│   ├── feature.h               # Image feature (patch)
│   ├── common_lib.h            # Shared types: StatesGroup, pointWithVar, LidarMeasureGroup, enums
│   └── utils/
│       ├── types.h             # Eigen/PCL type aliases
│       ├── so3_math.h          # SO(3) Exp/Log helpers
│       └── color.h             # ANSI color macros
├── src/                        # Implementations (.cpp)
│   ├── main.cpp                # Entry point: initializes ROS node & LIVMapper
│   ├── LIVMapper.cpp           # Main SLAM loop (sync, LIO, VIO, publish)
│   ├── voxel_map.cpp           # Voxel octree building, plane fitting, residual computation
│   ├── IMU_Processing.cpp      # IMU initialization, forward propagation, point undistortion
│   ├── preprocess.cpp          # LiDAR feature extraction (edge/plane points)
│   ├── vio.cpp                 # VIO: patch warp, Jacobian, EKF update, visual map management
│   ├── frame.cpp               # Frame constructor & image pyramid
│   └── visual_point.cpp        # Visual point management (obs, ref patch)
├── config/                     # YAML configs per sensor suite
│   ├── avia.yaml               # Livox Avia + camera
│   ├── HILTI22.yaml            # HILTI 2022
│   ├── MARS_LVIG.yaml          # MARS LVIG
│   └── NTU_VIRAL.yaml          # NTU VIRAL
├── launch/                     # ROS launch files
├── rviz_cfg/                   # RViz config files
├── scripts/                    # Utility scripts (colmap_output.sh, mesh.py)
├── third_party/                # Vendored deps: Sophus, rpg_vikit
│   ├── Sophus/                 # SO(3)/SE(3) Lie group algebra (libSophus.so)
│   └── rpg_vikit/              # Camera models, math utils (catkin package)
├── Dockerfile                  # ROS Noetic container build
├── docker-compose.yml          # Podman/Docker Compose config
├── run.sh                      # Container runner (build/shell/launch/rviz/bag)
├── CMakeLists.txt              # Build: 5 libs + 1 executable
└── package.xml                 # ROS package manifest
```

## Key Architecture

- **Node**: Single ROS node (`fastlivo_mapping` from `src/main.cpp`)
- **Orchestrator**: `LIVMapper` — subscribes to LiDAR, IMU, camera topics; synchronizes measurements; calls LIO (`VoxelMapManager`) and VIO (`VIOManager`)
- **SLAM Modes**: `ONLY_LO` (0), `ONLY_LIO` (1), `LIVO` (2)
- **EKF State**: 19-dimensional (`DIM_STATE = 19`): rot(3) + pos(3) + inv_expo(1) + vel(3) + bias_g(3) + bias_a(3) + gravity(3)

## Build & Run

```bash
# Native (ROScatkin workspace)
catkin_make
roslaunch fast_livo mapping_avia.launch
rosbag play dataset.bag

# Containerized (recommended)
./run.sh build
./run.sh launch mapping_avia.launch
./run.sh bag /path/to/dataset.bag
```

## Code Conventions

- **Header guard**: `#ifndef NAME_H` / `#define NAME_H` (no `#pragma once`)
- **License block**: Every file starts with the FAST-LIVO2 license header
- **Eigen alignment**: Classes with Eigen members must include `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`
- **Type aliases** (from `utils/types.h`):
  - `V3D` = `Eigen::Vector3d`, `M3D` = `Eigen::Matrix3d`
  - `PointCloudXYZI` = `pcl::PointCloud<pcl::PointXYZINormal>`
  - `PointType` = `pcl::PointXYZINormal`
  - `MD(a,b)` = `Eigen::Matrix<double, a, b>`
- **Core structs** (from `common_lib.h`): `StatesGroup`, `pointWithVar`, `LidarMeasureGroup`, `MeasureGroup`
- **Smart pointers**: `VIOManagerPtr` (`std::shared_ptr<VIOManager>`), `VoxelMapManagerPtr`, `ImuProcessPtr`, `PreprocessPtr`, `FramePtr` (`std::unique_ptr<Frame>`)
- **File I/O**: Logs written to `Log/` directory via `DEBUG_FILE_DIR(name)` macro (= `ROOT_DIR/Log/name`)
- **Config**: ROS parameter server loaded in `LIVMapper::readParameters()`
- **Publishing**: `LIVMapper` has dedicated `publish_*` methods for odometry, path, point clouds, images

## Common Tasks

- **Add a new LiDAR driver**: Define point struct in `preprocess.h`, add handler, register in `Preprocess::process()`, add enum to `LID_TYPE`
- **Modify EKF dimension**: Update `DIM_STATE` in `common_lib.h`, adjust `StatesGroup` operators, update Jacobian shapes in `vio.cpp` / `voxel_map.cpp`
- **Add config parameter**: Add to YAML, read in `LIVMapper::readParameters()`, store as member
- **New launch profile**: Create YAML in `config/`, create launch file in `launch/`, add RViz config in `rviz_cfg/`