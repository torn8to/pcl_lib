# WARP.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Common commands

- Configure and build (Release):

```bash path=null start=null
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

- Debug build:

```bash path=null start=null
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
```

- Clean rebuild:

```bash path=null start=null
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

- Specify CUDA architectures (optional, if you need a specific GPU arch):

```bash path=null start=null
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES="75;86;89"
cmake --build build -j
```

Notes:
- Tests/lint: No CTest targets or lint configs are defined. GoogleTest is fetched but no tests are registered.
- Build output: the shared library target is `pcl_lib_lib` (e.g., Linux artifact under `build/`).

## High-level architecture

This is a C++17/CUDA project built with CMake. It implements a voxelized point-cloud map and ICP-based registration, with an (unwired) ROS 2 node and experimental CUDA implementations.

- Core data structures
  - `src/VoxelMap.hpp, src/VoxelMap.cpp`: Voxelized map backed by `std::unordered_map<Voxel, std::vector<Eigen::Vector3d>>` with:
    - Insertion capped per-voxel; spacing threshold avoids near-duplicates.
    - Neighbor query across the 27-adjacent voxel neighborhood for first nearest neighbor.
    - Range-based pruning and optional LFU-based pruning (`src/LFUCache.hpp`).
    - Utility transforms and full cloud extraction.
  - `src/PointToVoxel.hpp`, `src/VoxelUtils.hpp`, `src/PointUtils.hpp`: Helpers for voxel indexing, downsampling, and transforms.

- Registration (CPU, TBB)
  - `src/Registration.hpp, src/Registration.cpp`:
    - Transforms source points by current estimate; associates correspondences via `VoxelMap::firstNearestNeighborQuery` with a distance threshold.
    - Builds normal equations using a point-to-point Jacobian (`Eigen::Matrix36d`) and a Geman–McClure robust kernel; solves via LDLT; iterates until `convergence_`.
    - Parallelized with oneTBB (`tbb::parallel_for`, `tbb::parallel_reduce`).

- Pipeline orchestration
  - `src/Pipeline.hpp, src/Pipeline.cpp`:
    - Config-driven pipeline; optionally downsamples differently for odometry vs mapping.
    - Uses `AdaptiveThreshold` (from KISS-ICP) to set correspondence radius each frame.
    - Runs registration against the current `VoxelMap`, updates pose and map, prunes far voxels.

- ROS 2 node (not wired in CMake)
  - `src/lid_odom_node.cpp`: Wraps the pipeline in an `rclcpp::Node`; subscribes to `sensor_msgs::msg::PointCloud2`, publishes odometry and debug clouds; performs optional motion de-skew (`src/MotionCompensation.hpp`) and ROS conversions (`src/Convert.hpp`, `src/tf2_sophus.hpp`).
  - To run as a node, a ROS 2 package/CMake target needs to be added (not present in the current `CMakeLists.txt`).

- Experimental CUDA path
  - `src/cuda/GPUVoxelHashMap.cuh/.cu`, `src/cuda/GPURegistration.cuh/.cu` use Thrust and cuCollections (`cuco::dynamic_map`) to prototype GPU voxel map and ICP.
  - These files are compiled into the shared library; the GPU path is not yet integrated with the CPU `Pipeline` and contains TODOs/rough edges.

## Build system and dependencies

- CMake: top-level `CMakeLists.txt` sets C++17 and CUDA, pulls dependencies via CPM/FetchContent.
- Required packages: `Eigen3`, `Sophus`, `CUDA`/`CUDAToolkit`. TBB is used in code but not explicitly linked in CMake.
- External deps fetched in build: `NVIDIA/cuCollections` (via CPM), `googletest` (not used yet).
