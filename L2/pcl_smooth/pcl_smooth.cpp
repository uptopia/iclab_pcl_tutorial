#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/surface/mls.h>
#include<pcl/visualization/pcl_visualizer.h>

using namespace std;
typedef pcl::PointXYZ PointT;

int main()
{
    //===========//
    // Parameters
    //===========//
    std::string file_load_path = "/home/upup/iclab_pcl_tutorial/example_data/scene_D405_cut.pcd";
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_up(new pcl::PointCloud<PointT>);

    //===========//
    // Load PCD
    //===========//     
    if(pcl::io::loadPCDFile<PointT>(file_load_path, *cloud)== -1)
    {
        PCL_ERROR("Failed to load file %s", file_load_path);
        system("pause");
        return -1;
    }

    //(1) Voxel Grid
    float leafsize = 0.005;
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*cloud);

    //================//
    // MLS Smoothing
    //================//
    //https://pcl.readthedocs.io/projects/tutorials/en/latest/resampling.html

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
    
    pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.08);
    mls.process(*mls_points);

    //===============//
    // Visualization
    //===============//
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("upsampling"));
    viewer->initCameraParameters();
    int v1(0),v2(0);

    // viewport 1: ori + mls_smooth
    viewer->createViewPort(0.0,0.0,0.5,1.0,v1); //(Xmin,Ymin,Xmax,Ymax)設置窗口座標
    viewer->setBackgroundColor(0,0,0,v1);       //設置背景
    viewer->addText("original clouds (wh); smoothed cloud (gn)", 10,10,"v1 text", v1);
    viewer->addPointCloud<PointT>(cloud, pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud, 255,255,255), "cloud1", v1);
    viewer->addPointCloud<pcl::PointNormal>(mls_points, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(mls_points, 0, 255, 0), "mls_points", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"mls_points");

    //viewport 2: mls_smooth
    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
    viewer->setBackgroundColor(0.3,0.3,0.3,v2);
    viewer->addText("smoothed cloud, total pts = " + to_string(mls_points->size()), 10,10,"mls_points text", v2);
    viewer->addPointCloud<pcl::PointNormal>(mls_points,"mls_points_only", v2);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce();
    }

    return 0;
}