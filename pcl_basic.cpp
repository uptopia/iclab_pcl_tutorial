#include<iostream>

#include<pcl/io/pcd_io.h>
#include<pcl/common/common.h>
#include<pcl/common/centroid.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/filters/filter.h> //removeNan
using namespace std;

pcl::PointXYZ PointT;

int main()
{
    //===========//
    // Parameters
    //===========//
    file_load_path_cloud1 = "";
    file_load_path_cloud2 = "";
    file_save_path = "";
    file_save_pathASCII = "";
     
    pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    
    //===========//
    // Load PCD
    //===========//
    if(pcl::io::loadPCDFile<PointT>(file_load_path_cloud1, *cloud1)== -1)
    {
        PCL_ERROR("Failed to load file" + file_load_path_cloud1);
        system("pause");
        return -1;
    }
    if(pcl::io::loadPCDFile<PointT>(file_load_path_cloud2, *cloud2)== -1)
    {
        PCL_ERROR("Failed to load file" + file_load_path_cloud2);
        system("pause");
        return -1;
    }

    //===========//
    // Cloud INFO
    //===========//
    //https://pointclouds.org/documentation/tutorials/basic_structures.html
    int width = cloud1->width;
    int height = cloud1->height;
    int size = cloud1->size;
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
        if(!pcl_isfinite(cloud1->at(i)))//skipping Nans
        {
            continue;
        }
    }
    //(2)removeNaNFromPointCloud
    cloud1->is_dense = false;
    std::vector<int> nan_indices;
    pcl::removeNaNFrom PointCloud(*cloud1, *cloud1, nan_indices);

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
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->initCameraParameters();
    int v1(0),v2(0);;

    viewer->createViewPort(0.0,0.0,0.5,1.0,v1);//(Xmin,Ymin,Xmax,Ymax)設置窗口座標
    viewer->setBackgroundColor(0,0,0,v1);//設置背景
    viewer->addText("original clouds", 10,10,"v1 text", v1);//設置視口名稱

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud1_color(cloud1, 0, 255, 0); // green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud2_color(cloud2, 255, 0, 0); // green
    
    viewer->addPointCloud<pcl::PointXYZ>(cloud1, cloud1_color, "cloud1", v1);//添加點雲
    viewer->addPointCloud<pcl::PointXYZ>(cloud2, cloud2_color, "cloud2", v1);//添加點雲
	
    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
    viewer->setBackgroundColor(0.3,0.3,0.3,v2);
    viewer->addText("combined cloud", 10,10,"v2 text", v2);
    viewer->addPointCloud(cloud_combined,"cloud_combined", v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE,3,"cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE,3,"cloud2");
    while (!viewer->wasStopped ())
    {

     viewer->spinOnce();
    }


    //===========//
    // Save PCD
    //===========//
    pcl::savePCDFile(file_save_path, *cloud);
    pcl::savePCDFileASCII(file_save_path, *cloud);
    
    
    return 0;
}