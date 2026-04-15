<span style="color: orange;">Modifications are underway.</span>


# *Iris*
* Visual localization in pre-build pointcloud maps.
* ~~**OpenVSLAM** and **VINS-mono**  can be used.~~ <span style="color: orange;">Modifications are underway.</span>


## Video
[![](https://img.youtube.com/vi/a_BnifwBZC8/0.jpg)](https://www.youtube.com/watch?v=a_BnifwBZC8)


## Submodule 
* [OpenVSLAM forked by MapIV](https://github.com/MapIV/openvslam.git)
> ~~[original repository (xdspacelab)](https://github.com/xdspacelab/openvslam)~~


## Dependency
If you are using ROS, you only need to install `g2o` and `DBoW2`.
* [ROS](http://wiki.ros.org/)
* [OpenCV](https://opencv.org/) >= 3.2
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) 
* [PCL](https://pointclouds.org/)
* [g2o](https://github.com/RainerKuemmerle/g2o)
* [DBow2](https://github.com/shinsumicco/DBoW2.git)
  * Please use the custom version released in [https://github.com/shinsumicco/DBoW2](https://github.com/shinsumicco/DBoW2)

> ~~see also: [openvslam](https://openvslam.readthedocs.io/en/master/installation.html#dependencies).~~ <span style="color: orange;">Modifications are underway.</span>

## How to Build

### ROS 1 (Noetic) — devcontainer

Open `ros1/src/iris/` in VS Code and reopen in the devcontainer.  
The container builds g2o and DBoW2 automatically, then runs `catkin_make`.

Or manually:
```bash
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone --recursive https://github.com/rsasaki0109/iris.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### ROS 2 (Humble) — devcontainer

Open the repository in VS Code and reopen in the devcontainer (select `ros2/.devcontainer`).  
The container builds g2o and DBoW2 automatically, then runs `colcon build`.

Or manually:
```bash
git clone --recursive https://github.com/rsasaki0109/iris.git ~/iris_repo
mkdir -p ~/colcon_ws && cd ~/colcon_ws
colcon build \
  --base-paths ~/iris_repo/ros2/iris ~/iris_repo/ros2/openvslam_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## How to Run with Sample Data

### Download sample data
1. Vocabulary: `orb_vocab.dbow2` from [Dropbox](https://www.dropbox.com/s/z8vodds9y6yxg0p/orb_vocab.dbow2?dl=0)
2. Pointcloud map: `kitti_00.pcd` from [Dropbox](https://www.dropbox.com/s/tzdqtsl1p7v1ylo/kitti_00.pcd?dl=0)
3. Rosbag: `kitti_00_stereo.bag` from [Dropbox](https://www.dropbox.com/s/kfouz9gkjefpvb5/kitti_00_stereo.bag?dl=0)

### ROS 1

#### Stereo
```bash
roslaunch iris stereo_kitti00.launch
roslaunch iris rviz.launch          # another terminal
rosbag play kitti_00_stereo.bag     # another terminal
```

#### Monocular
```bash
roslaunch iris mono_kitti00.launch
roslaunch iris rviz.launch          # another terminal
rosbag play kitti_00_stereo.bag     # another terminal
```

### ROS 2

Place `orb_vocab.dbow2` and `kitti_00.pcd` in `~`.

#### Stereo
```bash
ros2 launch iris stereo_kitti00.launch.py \
  pcd_path:=$HOME/kitti_00.pcd \
  vocab_file:=$HOME/orb_vocab.dbow2
ros2 launch iris rviz.launch.py     # another terminal
ros2 bag play kitti_00_stereo       # another terminal
```

#### Monocular
```bash
ros2 launch iris mono_kitti00.launch.py \
  pcd_path:=$HOME/kitti_00.pcd \
  vocab_file:=$HOME/orb_vocab.dbow2
ros2 launch iris rviz.launch.py     # another terminal
ros2 bag play kitti_00_stereo       # another terminal
```

> If the estimated position is misaligned, use `2D Pose Estimate` in RViz2.

## How to Run with Your Data

```bash
# ROS 1
roslaunch iris openvslam.launch iris_config_path:=<your_iris.yaml>

# ROS 2
ros2 launch iris stereo_kitti00.launch.py \
  iris_config_path:=<your_iris.yaml> \
  vslam_config_path:=<your_vslam.yaml> \
  pcd_path:=<your_map.pcd> \
  vocab_file:=<your_vocab.dbow2>
```

## License
~~Iris is provided under the BSD 3-Clause License.~~
<span style="color: orange;">Modifications are underway.</span>

The following files are derived from third-party libraries.
* `iris/src/optimize/types_gicp.hpp` : part of [g2o](https://github.com/RainerKuemmerle/g2o) (BSD)
* `iris/src/optimize/types_gicp.cpp` : part of [g2o](https://github.com/RainerKuemmerle/g2o) (BSD)
* `iris/src/pcl_/correspondence_estimator.hpp` : part of [pcl](https://github.com/PointCloudLibrary/pcl) (BSD)
* `iris/src/pcl_/correspondence_estimator.cpp` : part of [pcl](https://github.com/PointCloudLibrary/pcl) (BSD)
* `iris/src/pcl_/normal_estimator.hpp` : part of [pcl](https://github.com/PointCloudLibrary/pcl) (BSD)
* `iris/src/pcl_/normal_estimator.cpp` : part of [pcl](https://github.com/PointCloudLibrary/pcl) (BSD)


## Reference
- T. Caselitz, B. Steder, M. Ruhnke, and W. Burgard, “Monocular camera localization in 3d lidar maps,” in 2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2016, pp. 1926–1931.
> http://www.lifelong-navigation.eu/files/caselitz16iros.pdf
