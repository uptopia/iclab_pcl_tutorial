// CMakeLists.txt
// cmake ..
// make
// ./region_grow

#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/passthrough.h>
#include<pcl/search/kdtree.h>
#include<pcl/features/normal_3d.h>
#include<pcl/segmentation/region_growing.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
typedef pcl::PointXYZ PointT;

using namespace std;

int main()
{
    std::string file_path = "/home/upup/iclab_pcl_tutorial/L4/sample_scene/scene_organized_cloud.pcd";  
    
    pcl::PointCloud<PointT>::Ptr scene_ori(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);

    //===========//
    // Load PCD
    //===========//
    if(pcl::io::loadPCDFile<PointT>(file_path, *scene_ori)== -1)
    {
        std::string error_msg = "Failed to load file" + file_path; 
        PCL_ERROR("Failed to load file", file_path.c_str());
        system("pause");
        return -1;
    }   
    
    //===========//
    // Preprocess
    //===========//
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (scene_ori);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.4, 0.977);
    pass.filter(*scene_ori);// (*indices); //去除不相關的場景

    //(2)removeNaNFromPointCloud
    scene->is_dense = false;
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*scene_ori, *scene_ori, nan_indices);

    //(1) Voxel Grid
    float leafsize = 0.005;
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(scene_ori);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*scene);


    pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(scene);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::RegionGrowing<PointT, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (300000); //限定每個分割的點數 有桌子所以要調大
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (scene);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (6.0 / 180.0 * M_PI); //更改可接受幅度
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    reg.
    
    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
    std::cout << "These are the indices of the points of the initial" <<
        std::endl << "cloud that belong to the first cluster:" << std::endl;
    int counter = 0;
    while (counter < clusters[0].indices.size ())
    {
        std::cout << clusters[0].indices[counter] << ", ";
        counter++;
        if (counter % 10 == 0)
        std::cout << std::endl;
    }
    std::cout << std::endl;




    //===============//
    // Visualization
    //===============//
    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("RegionGrow"));
    // viewer->addPointCloud<PointT>(scene,"scene");
    // viewer->spin();

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped ())
    {
    }

    return 0;
}