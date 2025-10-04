# RelocAlign

Collection of PointCloud Alignment Methods.

* ICP, NDT from PointCloud Library
* GICP, VGICP, NDT and CUDA variants from [fast_gicp](https://github.com/koide3/fast_gicp.git)

## Installation

### Dependencies

- PCL
- Eigen
- OpenMP
- CUDA (optional)
- [Sophus](https://github.com/strasdat/Sophus)
- [nvbio](https://github.com/NVlabs/nvbio)

### ROS

```shell
mkdir relocalign_ws && cd relocalign_ws
mkdir src && cd src

git clone https://github.com/RightTr/RelocAlign.git
cd RelocAlign

git submodule update --init --reursive

cd ../..

catkin_make

# Build with CUDA

catkin_make -DBUILD_VGICP_CUDA=ON
```
