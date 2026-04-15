This is my lidar odometry workspace contains a few different things:

Gpu registration implementations for point to point icp 

Contains:
- iteratve and non iterative ekf implementation
- a tested cuda implementation of hash map using cuda memory primitives
- GPU and CPU implementations of Sparse Voxel Maps
- CPU dense voxel map 
- offline visualiztion processing bags and data analysis


## Difference between cpu and gpu Voxel hash maps
GPU Point count is templated requiring the amount of points in each voxel to be decided at compile time. Unlike that the cpu version can be decided at runtime.

GPU version is not vectorized as we use Eigen as the Math library but macros for vectorizing on the cpu do not apply on the GPU, hence all types are Declared with the `Eigen::DontAlign` template arguments.
