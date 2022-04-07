```
修改CMakeLists.txt中C++編譯版本(Line 5)

$ cd ~/iclab_pcl_tutorial/L1/src/ros_pcl_sub_pub
$ gedit CMakeLists.txt
#ROS Kinetic
add_compile_options(-std=c++11)
#ROS Noetic
add_compile_options(-std=c++14) 

$ cd ~/iclab_pcl_tutorial/L1
$ catkin_make


執行程式
<terminal 1>
$ cd ~/realsense_ros
$ . devel/setup.bash
$ roslaunch realsense2_camera rs_rgbd.launch

<terminal 2>
$ cd ~/iclab_pcl_tutorial/L1
$ . devel/setup.bash
$ rosrun ros_pcl_sub_pub ros_pcl_sub_pub
```