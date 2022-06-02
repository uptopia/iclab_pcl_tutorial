#include<iostream>

#include<pcl/io/pcd_io.h>
#include<pcl/common/common.h>
#include<pcl/common/centroid.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/filters/filter.h> //removeNan
using namespace std;

typedef pcl::PointXYZ PointT;

int main()
{
    //===========//
    // Parameters
    //===========//
    std::string file_load_path_cloud1 = "/home/upup/iclab_pcl_tutorial/example_data/scene_D405_cut.pcd";
    std::string file_load_path_cloud2 = "/home/upup/iclab_pcl_tutorial/example_data/scene_D435i.pcd";
    std::string file_save_path = "/home/upup/iclab_pcl_tutorial/example_data/cloud_binary.pcd";
    std::string file_save_pathASCII = "/home/upup/iclab_pcl_tutorial/example_data/cloud_ascii.pcd";
     
    pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    
    //===========//
    // Load PCD
    //===========//
    if(pcl::io::loadPCDFile<PointT>(file_load_path_cloud1, *cloud1)== -1)
    {
        PCL_ERROR("Failed to load file %s", file_load_path_cloud1);
        system("pause");
        return -1;
    }
    if(pcl::io::loadPCDFile<PointT>(file_load_path_cloud2, *cloud2)== -1)
    {
        PCL_ERROR("Failed to load file %s", file_load_path_cloud2);
        system("pause");
        return -1;
    }

    //===========//
    // Cloud INFO
    //===========//
    //https://pointclouds.org/documentation/tutorials/basic_structures.html
    int width = cloud1->width;
    int height = cloud1->height;
    int size = cloud1->size();
    bool is_dense = cloud1->is_dense;    //True: No NaN/Inf, all points are finit
    Eigen::Vector4f ori_translation = cloud1->sensor_origin_;
    Eigen::Quaternionf ori_orientation = cloud1->sensor_orientation_;

    //===================//
    // remove NaN
    //===================//
    //https://blog.csdn.net/weixin_42657460/article/details/118942759 //important!!!!
    //https://blog.csdn.net/qq_36501182/article/details/79170438
    //(1)pcl_isfinite
    for(int i =0; i<cloud1->size(); i++)
    {
        PointT pt = cloud1->points[i];
        //skipping Nans
        if(!pcl_isfinite(pt.x) || !pcl_isfinite(pt.y) || !pcl_isfinite(pt.z))
        {
            continue;
        }
    }
    //(2)removeNaNFromPointCloud
    cloud1->is_dense = false;
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*cloud1, *cloud1, nan_indices);

    //===================//
    // Center & Centroid
    //===================//
    //https://blog.csdn.net/weixin_46098577/article/details/119981408
    //https://blog.csdn.net/qq_36501182/article/details/79005933
    PointT min_point, max_point, center_point;    
    pcl::getMinMax3D(*cloud1, min_point, max_point);
    center_point.x = (min_point.x + max_point.x)/2;
    center_point.y = (min_point.y + max_point.y)/2;
    center_point.z = (min_point.z + max_point.z)/2;

    cout <<"Min (x, y, z) = " << min_point.x <<", "<< min_point.y <<", "<< min_point.z << endl
         <<"Max (x, y, z) = " << max_point.x <<", "<< max_point.y <<", "<< max_point.z << endl;
    cout <<"Center Point: " << center_point <<endl;
    
    Eigen::Vector4f centroid;
    PointT centroid_point;
    pcl::compute3DCentroid(*cloud1, centroid);
    centroid_point = pcl::PointXYZ(centroid[0], centroid[1], centroid[2]);
    cout <<"Centroid Point: " << centroid[0] <<", "<< centroid[1] <<", "<< centroid[2] <<endl;

    //===========//
    // Add Clouds
    //===========//
    pcl::PointCloud<PointT>::Ptr cloud_combined(new pcl::PointCloud<PointT>);
    *cloud_combined = *cloud1 + *cloud2;

    //===============//
    // Visualization
    //===============//
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("add 2 clouds"));
    viewer->initCameraParameters();
    int v1(0),v2(0);

    // viewport 1: ori + voxel
    viewer->createViewPort(0.0,0.0,0.5,1.0,v1); //(Xmin,Ymin,Xmax,Ymax)設置窗口座標
    viewer->setBackgroundColor(0,0,0,v1);       //設置背景
    viewer->addText("original clouds (wh); added cloud (rd)", 10,10,"v1 text", v1);
    viewer->addPointCloud<PointT>(cloud2, pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud2, 255, 255, 255), "cloud2", v1);
    viewer->addPointCloud<PointT>(cloud1, pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud1, 255, 0, 0), "cloud1", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"cloud1");

    //viewport 2: voxel
    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
    viewer->setBackgroundColor(0.3,0.3,0.3,v2);
    viewer->addText("added cloud, total pts = " + to_string(cloud1->size()), 10,10,"cloud1 text", v2);
    viewer->addPointCloud(cloud1,"cloud_filtered_only", v2);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce();
    }

    //===========//
    // Save PCD
    //===========//
    pcl::io::savePCDFileBinary(file_save_path, *cloud2);
    pcl::io::savePCDFileASCII(file_save_pathASCII, *cloud2);
    
    return 0;
}