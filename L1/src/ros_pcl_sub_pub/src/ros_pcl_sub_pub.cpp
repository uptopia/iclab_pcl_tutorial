//CMakeLists.txt
//要改add_compile_options(-std=c++11)
//add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/ros_pcl_sub_pub.cpp)
//target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

//realsense-ros
//. devel/setup.bash
//roslaunch realsense2_camera rs_rgbd.launch filters:=pointcloud depth_width:=1280 depth_height:=720 color_width:=1280 color_height:=720

//catkin_make
//. devel/setup.bash
//rosrun ros_pcl_sub_pub ros_pcl_sub_pub


//#https://github.com/methylDragon/pcl-ros-tutorial/blob/master/Starter%20Code/minimal_pcl/src/minimal_pub_sub.cpp
// ========================//
//   Save Realsense Data
// ========================//
// written by Shang-Wen, Wong.
// 2021.3.29

#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> //"pcl::fromROSMsg"

//realsense_msgs
//"~/realsense_ros/devel/include/realsense2_camera/Extrinsics.h"
// #include <realsense2_camera/Extrinsics.h> 

// pcl
#include <pcl/io/pcd_io.h>

// C++
#include <iostream>

using namespace std;


typedef pcl::PointXYZRGB PointTRGB;

bool save_organized_cloud = true;
bool save_unorganized_cloud = true;

std::string file_path_cloud_organized = "organized_cloud_tmp.pcd";
std::string file_path_cloud_unorganized = "unorganized_cloud_tmp.pcd";

pcl::PointCloud<PointTRGB>::Ptr organized_cloud_ori(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr unorganized_cloud_ori(new pcl::PointCloud<PointTRGB>);

// void depth_to_color_extrinsic_cb (const realsense2_camera::Extrinsics& extrinsicsMsg)
// {
//     //=================================================================//
//     // Transformation from realsense depth to color coordinate system
//     // Subscribe "/camera/extrinsic/depth_to_color" topic
//     //=================================================================//
//     cout << "\ndepth_to_color_extrinsic_cb" << endl;

//     Eigen::Matrix4f matrix_depth2color = Eigen::Matrix4f::Identity();    
//     matrix_depth2color = Eigen::Matrix4f::Identity();
//     matrix_depth2color <<
//     extrinsicsMsg.rotation[0], extrinsicsMsg.rotation[1], extrinsicsMsg.rotation[2], extrinsicsMsg.translation[0],
//     extrinsicsMsg.rotation[3], extrinsicsMsg.rotation[4], extrinsicsMsg.rotation[5], extrinsicsMsg.translation[1],
//     extrinsicsMsg.rotation[6], extrinsicsMsg.rotation[7], extrinsicsMsg.rotation[8], extrinsicsMsg.translation[2],
//     0.0, 0.0, 0.0, 1.0;
    
//     cout << "frame id = " << extrinsicsMsg.header.frame_id << endl;
//     cout << "matrix_depth2color = \n" << matrix_depth2color << endl;
// }

void organized_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& organized_cloud_msg)
{
    //==================================================//
    // 有序點雲 Organized Point Cloud; Depth Point Cloud
    // Subscribe "/camera/depth_registered/points" topic
    //==================================================//
    cout << "organized_cloud_cb" << endl;

    int height = organized_cloud_msg->height;
    int width = organized_cloud_msg->width;
    int points = height * width;

    if((points!=0) && (save_organized_cloud ==true))
    {
        // 將點雲格式由sensor_msgs/PointCloud2轉成pcl/PointCloud(PointXYZ, PointXYZRGB)
        organized_cloud_ori->clear();
        pcl::fromROSMsg(*organized_cloud_msg, *organized_cloud_ori);

        cout << "organized_cloud_ori saved: " << file_path_cloud_organized << "; (width, height) = " << width << ", " << height << endl;
        pcl::io::savePCDFileBinary<PointTRGB>(file_path_cloud_organized, *organized_cloud_ori); //savePCDFileASCII
        cout << "organized_cloud_ori saved: DONE! \n";
    }
}

void unorganized_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& unorganized_cloud_msg)
{
    //==================================================//
    // 無序點雲 Unorganized Point Cloud; RGB Point Cloud
    // Subscribe "/camera/depth/color/points" topic
    //==================================================//
    cout << "unorganized_cloud_cb" << endl;

    int height = unorganized_cloud_msg->height;
    int width = unorganized_cloud_msg->width;
    int points = height * width;

    if((points!=0) && (save_unorganized_cloud ==true))
    {
        // 將點雲格式由sensor_msgs/PointCloud2轉成pcl/PointCloud(PointXYZ, PointXYZRGB)
        unorganized_cloud_ori->clear();
        pcl::fromROSMsg(*unorganized_cloud_msg, *unorganized_cloud_ori);
        
        cout << "unorganized_cloud_ori saved: " << file_path_cloud_unorganized << "; (width, height) = " << width << ", " << height << endl;
        pcl::io::savePCDFileBinary<PointTRGB>(file_path_cloud_unorganized, *unorganized_cloud_ori); //savePCDFileASCII
        cout << "unorganized_cloud_ori saved: DONE!\n";

    }    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_realsense_data");

    ros::NodeHandle nh;

    // //深度相機座標系到彩色相機座標系的轉換關係 Transformation from depth to color coord. sys.
    // ros::Subscriber sub_cam_extrinsic = nh.subscribe("/camera/extrinsics/depth_to_color", 1, depth_to_color_extrinsic_cb);  //depth_to_color_trans_matrix
    
    //有序點雲 Organized Point Cloud 
    ros::Subscriber sub_organized_cloud = nh.subscribe("/camera/depth_registered/points", 1, organized_cloud_cb);
    
    //無序點雲 Unorganized Point Cloud (height = 1)
    ros::Subscriber sub_unorganized_cloud = nh.subscribe("/camera/depth/color/points", 1, unorganized_cloud_cb);

    ros::spin();

    return 0;
}