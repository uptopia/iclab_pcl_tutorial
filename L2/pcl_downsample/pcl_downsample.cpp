#include<iostream>
#include<pcl/io/pcd_io.h>

#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/uniform_sampling.h>
#include<pcl/visualization/pcl_visualizer.h>

using namespace std;
typedef pcl::PointXYZ PointT;

int main()
{
    //===========//
    // Parameters
    //===========//
    std::string file_load_path = "../../../example_data/scene_D435i.pcd";
    // std::string file_save_path = "";
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_voxel(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_uni_samp(new pcl::PointCloud<PointT>);

    //===========//
    // Load PCD
    //===========//     
    if(pcl::io::loadPCDFile<PointT>(file_load_path, *cloud)== -1)
    {
        PCL_ERROR("Failed to load file %s", file_load_path);
        system("pause");
        return -1;
    }

    //==============//
    // Downsampling
    //==============//
    //(1) Voxel Grid
    float leafsize = 0.01;
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*cloud_voxel);

    //(2) Uniform Sampling
    //https://www.cnblogs.com/li-yao7758258/p/6527969.html 上採樣
    pcl::PointCloud<int> keypoint_indices;
    pcl::UniformSampling<PointT> us;
    us.setInputCloud(cloud);
    us.setRadiusSearch(0.01f);
    us.filter(*cloud_uni_samp);
    // pcl::copyPointCloud(*cloud, keypoint_indices.points, *cloud_voxel);

    //===============//
    // Visualization
    //===============//
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->initCameraParameters();
    int v1(0),v2(0), v3(0), v4(0);

    // viewport 1: ori + voxel
    viewer->createViewPort(0.0,0.5,0.5,1.0,v1); //(Xmin,Ymin,Xmax,Ymax)設置窗口座標
    viewer->setBackgroundColor(0,0,0,v1);       //設置背景
    viewer->addText("original clouds (gn); voxel grid cloud (rd)", 10,10,"v1 text", v1);
    viewer->addPointCloud<PointT>(cloud, pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud, 0,255,0), "cloud1", v1);
    viewer->addPointCloud<PointT>(cloud_voxel, pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_voxel, 255, 0, 0), "cloud_voxel", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"cloud_voxel");

    //viewport 2: voxel
    viewer->createViewPort(0.5,0.5,1.0,1.0,v2);
    viewer->setBackgroundColor(0.3,0.3,0.3,v2);
    viewer->addText("downsampled cloud (voxel grid), total pts = " + to_string(cloud_voxel->size()), 10,10,"cloud_voxel text", v2);
    viewer->addPointCloud(cloud_voxel,"cloud_voxel_only", v2);

    //viewport 3: ori + uni
    viewer->createViewPort(0.0,0.0,0.5,0.5,v3); //(Xmin,Ymin,Xmax,Ymax)設置窗口座標
    viewer->setBackgroundColor(0,0,0,v3);       //設置背景
    viewer->addText("original clouds (gn); uniform sampling cloud (rd)", 10,10,"v2 text", v3);
    viewer->addPointCloud<PointT>(cloud, pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud, 0,255,0), "cloud_ori", v3);
    viewer->addPointCloud<PointT>(cloud_uni_samp, pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_uni_samp, 255, 0, 0), "cloud_uni_samp", v3);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"cloud_uni_samp");

    //viewport 4: uni
    viewer->createViewPort(0.5,0.0,1.0,0.5,v4);
    viewer->setBackgroundColor(0.3,0.3,0.3,v4);
    viewer->addText("downsampled cloud (uniform sampling), total pts = " + to_string(cloud_uni_samp->size()), 10,10,"cloud_uni_samp text", v4);
    viewer->addPointCloud(cloud_uni_samp,"cloud_uni_samp_only", v4);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce();
    }

    return 0;
}