# I.C.Lab PCL Tutorial

![Ubuntu18.04 PCL1.8.1](https://img.shields.io/badge/Ububtu_18.04-PCL_1.8.1-green.svg)

![Ubuntu20.04 PCL1.10.0](https://img.shields.io/badge/Ububtu_20.04-PCL_1.10.0-green.svg)


## Dependencies
3D Cameras
* Install [Realsense ROS](https://github.com/IntelRealSense/realsense-ros)
or
* Install [Azure Kinet SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK)
* Install [Azure Kinect ROS Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver)

## Software environment
* Ubuntu 18.04
* ROS Melodic (PCL 1.8.1)

## L1: 3D感測技術及點雲庫(PCL)介紹

### pcd_write

### pcl_basic

### ros_pcl_sub_pub

## L2: 點雲降採樣及濾波功能實作

### pcl_filters

## L3: 點雲特徵點提取及匹配

### pcl_registration_pipline

## L4: 點雲分割功能實作

### pcl_segmentation

rosbag
/camera/aligned_depth_to_color/camera_info
/camera/aligned_depth_to_color/image_raw #depth

/camera/color/camera_info
/camera/color/image_raw #RGB

/camera/depth/camera_info
/camera/depth/image_rect_raw #depth

/camera/depth/color/points #無序點雲 Unorganized Point Cloud (height = 1)
/camera/depth_registered/points #有序點雲 Organized Point Cloud

/camera/extrinsics/depth_to_color





rosbag錄製及播放
https://zhuanlan.zhihu.com/p/151444739

//開啟realsense拍攝scene
roslaunch realsense2_camera rs_rgbd.launch filters:=pointcloud depth_width:=1280 depth_height:=720 color_width:=1280 color_height:=720

//錄製rosbag
rosbag record -O scene.bag /camera/aligned_depth_to_color/camera_info /camera/aligned_depth_to_color/image_raw /camera/color/camera_info /camera/color/image_raw /camera/depth/camera_info /camera/depth/image_rect_raw /camera/depth/color/points /camera/depth_registered/points /camera/extrinsics/depth_to_color

//播放
rosbag play --pause scene.bag
rosbag play scene.bag
rosbag play -r 0.5 scene.bag

//確認topic
rostopic list

//rviz預覽
rviz

顯示點雲
Add PointCloud2
/camera/depth/color/points
/camera/depth_registered/points

更改Fixed Frame
camera_aligned_depth_to_color_frame
camera_color_frame
camera_color_optical_frame
camera_depth_frame
camera_depth_optical_frame
camera_link



