# ROS2 image publisher 

## Description

This is a ROS2 package created to publish **images** of some of the famous dataset

## Installation

Source ROS2 environment (tested with eloquent)
```bash
source /opt/ros/eloquent/setup.bash
```

Clone the repository and build it

```bash
git clone https://github.com/mirellameelo/dataset_publisher_ros2.git

cd <path_to_dataset_publisher_ros2>/src
colcon build
```

## Usage

### OpenvSLAM dataset 

Download one of the [**Fisheye camera** sequence](https://drive.google.com/drive/folders/1SVDsgz-ydm1pAbrdmhRQTmWhJnUl_xr8) or one of the [**Equirectangular camera** sequence](https://drive.google.com/drive/folders/1A_gq8LYuENePhNHsuscLZQPhbJJwzAq4) and run:


```bash
ros2 run publisher openvslam_video -m <path_to_video>/video.mp4
```

Images will be published in **/video/image_raw** topic and frame_id is set to **image**

### KITTI dataset

Download one of the [odometry dataset(grayscale)](https://drive.google.com/drive/folders/1SVDsgz-ydm1pAbrdmhRQTmWhJnUl_xr8) and run:


```bash
ros2 run publisher kitti_images -i <path_to_dataset>/<sequence_number>
```

Stereo images will be published in **/image/right** and **image/left** topics, and frame_id is set to **image**

### EuRoc dataset

Download one of the [sequence](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) (ASL Dataset Format) and run:


```bash
ros2 run publisher euroc_images -i <path_to_dataset>/mav0
```

Stereo images will be published in **/image/right** and **image/left** topics, and frame_id is set to **image**
