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

## Run

* Manual Reloc

```shell
rosrun relocalign reloc_manual_xyzquat <x> <y> <z> <qx> <qy> <qz> <qw>

rosrun relocalign reloc_manual_xyzyaw <x> <y> <z> <yaw>
```

* Cloud Alignment Test
  
```shell
rosrun relocalign demo_reloc <config> <sourece_pcd> <target_pcd>

# config: config/demo_relocalign.yaml
```

* Reloc_Livox

```shell
roslaunch relocalign reloc_livox.launch
```
