# ROS2 image publisher 

## Description

This is a ROS2 package created to publish **images** of some of the famous dataset

## Installation

Source ROS2 environment (tested with Eloquent and Dashing)
```bash
source /opt/ros/<distro>/setup.bash
```

Clone the repository and build it

```bash
git clone https://github.com/mirellameelo/dataset_publisher_ros2.git

cd <path_to_dataset_publisher_ros2>
colcon build
```

Source the environment

```bash
source <path_to_dataset_publisher_ros2>/install/setub.bash
```

## Usage

### OpenvSLAM dataset 

Download one of the [**Fisheye camera** sequence](https://drive.google.com/drive/folders/1SVDsgz-ydm1pAbrdmhRQTmWhJnUl_xr8) or one of the [**Equirectangular camera** sequence](https://drive.google.com/drive/folders/1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4) and run:

**OBS**: it should work with any .mp4 video. 


```bash
ros2 run publisher openvslam_video -m <path_to_video>/video.mp4
```

Images will be published in **camera/image_raw** topic and frame_id is set to **camera_link**

### KITTI dataset

Download one of the [odometry dataset(grayscale)](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) and run:


```bash
ros2 run publisher kitti_images -i <path_to_dataset>/<sequence_number>
```

Stereo images will be published in **camera/left/image_raw** and **camera/right/image_raw** topics, and frame_id is set to **camera_link**

### EuRoc dataset

Download one of the [sequence](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) (ASL Dataset Format) and run:


```bash
ros2 run publisher euroc_images -i <path_to_dataset>/mav0
```

Stereo images will be published in **camera/left/image_raw** and **camera/right/image_raw** topics, and frame_id is set to **camera_link**
