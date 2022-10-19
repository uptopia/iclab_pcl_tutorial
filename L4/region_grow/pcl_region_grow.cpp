// CMakeLists.txt
// cmake ..
// make
// ./region_grow

//https://cloud.tencent.com/developer/article/1475904

#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/passthrough.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/search/kdtree.h>
#include<pcl/features/normal_3d.h>
#include<pcl/segmentation/region_growing.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointT;

using namespace std;

//===================//
// Sort Cluster Size
//===================//
bool compareClusterSize(pcl::PointIndices& c1, pcl::PointIndices& c2)
{
    return c1.indices.size() > c2.indices.size();
}

int main()
{
    std::string file_path = "../../../example_data/scene_pc_organized_cloud.pcd";  
    
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

    //(1)removeNaNFromPointCloud
    scene->is_dense = false;
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*scene_ori, *scene_ori, nan_indices);

    //(2) Voxel Grid
    float leafsize = 0.005;
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(scene_ori);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*scene);

    //====================//
    // Normal Estimation
    //====================//
    pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(scene);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    // Display Normal Estimation Results
    pcl::visualization::PCLVisualizer::Ptr view_normal(new pcl::visualization::PCLVisualizer("Normal"));
    view_normal->addPointCloudNormals<PointT, pcl::Normal>(scene, normals, 10, 0.01, "normals"); //display normal for every 10 points, length 0.01m
    view_normal->addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0,0,255), "scene");
    view_normal->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene");
    view_normal->spin();

    //=================//
    // Region Growing
    //=================//
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

    //===================//
    // Sort Cluster Size
    // 將cluster由大到小排序
    //===================//
    sort(clusters.begin(), clusters.end(), compareClusterSize);

    for(int n = 0; n<clusters.size(); n++)
    {
        cout << "# "<< n <<": " << clusters[n].indices.size() << endl;
    }

    cout << "Total number of clusters is equal to " << clusters.size () << endl;
    cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
    cout << "These are the indices of the points of the initial" <<
        endl << "cloud that belong to the first cluster:" << endl;
    int counter = 0;
    while (counter < clusters[0].indices.size ())
    {
        cout << clusters[0].indices[counter] << ", ";
        counter++;
        if (counter % 10 == 0)
            cout << endl;
    }
    cout << endl;

   //==================================//
    // Visualization RegionGrow Result
    //==================================//
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::visualization::CloudViewer cluster_viewer ("Cluster viewer");
    cluster_viewer.showCloud(colored_cloud);
    while (!cluster_viewer.wasStopped ())
    {
    }

    //===============//
    // Visualization
    //===============//
    //https://www.cnblogs.com/li-yao7758258/p/6445127.html
    //https://github.com/PointCloudLibrary/pcl/issues/2049
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("RegionGrow"));
    pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(scene);
    extract.setNegative(false);
    
    for(int n=0; n<clusters.size(); n++)
    {
        inliers->indices = clusters[n].indices;
    
        cluster_cloud->clear();
        extract.setIndices(inliers);
        extract.filter(*cluster_cloud);

        viewer->removeAllPointClouds();
        viewer->addPointCloudNormals<PointT, pcl::Normal>(scene, normals, 10, 0.01, "normals");
        viewer->addPointCloud<PointT>(cluster_cloud, pcl::visualization::PointCloudColorHandlerCustom<PointT>(cluster_cloud, 255,255,0), "cluster_cloud");
        std::string text = "cluster #" + std::to_string(n);
        text = text + ": " + std::to_string(clusters[n].indices.size() ) + " points";
        if(n==0)
            viewer->addText(text, 20, 60, 25, 1.0 ,0.0, 0.0,"text");
        else
            viewer->updateText(text, 20, 60, 25, 1.0 ,0.0, 0.0, "text");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cluster_cloud");
        viewer->spin();
    }

    return 0;
}