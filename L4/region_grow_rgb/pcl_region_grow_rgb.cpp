// CMakeLists.txt
// cmake ..
// make
// ./region_grow

//https://stackoverflow.com/questions/16572666/using-clustered-indexes-from-point-cloud-in-rgb-image

#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/passthrough.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/search/kdtree.h>
#include<pcl/features/normal_3d.h>
#include<pcl/segmentation/region_growing_rgb.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGB PointTRGB;

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
    
    pcl::PointCloud<PointTRGB>::Ptr scene_ori(new pcl::PointCloud<PointTRGB>);
    pcl::PointCloud<PointTRGB>::Ptr scene(new pcl::PointCloud<PointTRGB>);

    //===========//
    // Load PCD
    //===========//
    if(pcl::io::loadPCDFile<PointTRGB>(file_path, *scene_ori)== -1)
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
    pcl::PassThrough<PointTRGB> pass;
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
    pcl::VoxelGrid<PointTRGB> vg;
    vg.setInputCloud(scene_ori);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*scene);

    //====================//
    // Region Growing RGB
    //====================//
    // RegionGrowRGB 和 RegionGrow的兩個差異點：
    // (1) 使用color，而非normal
    // (2) 使用merging algorithm來控制over-, under- segmentation
    // After the segmentation, an attempt for merging clusters with close colors is made. 
    // Two neighbouring clusters with a small difference between average color are merged together. 
    // Then the second merging step takes place. 
    // During this step every single cluster is verified by the number of points that it contains. 
    // If this number is less than the user-defined value 
    // than current cluster is merged with the closest neighbouring cluster.
    pcl::search::Search <PointTRGB>::Ptr tree (new pcl::search::KdTree<PointTRGB>);
    pcl::RegionGrowingRGB<PointTRGB> reg;
    reg.setInputCloud(scene);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(10);   //[Clustering]的閾值，判斷是否是鄰近點
    reg.setPointColorThreshold(3);  //[Clustering]的閾值，判對是否屬於同一群
    reg.setRegionColorThreshold(5); //[Merging]的閾值
    reg.setMinClusterSize(600);     //若cluster點數少於設定值，則與其他cluster合併

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);

    //===================//
    // Sort Cluster Size
    // 將cluster由大到小排序
    //===================//
    sort(clusters.begin(), clusters.end(), compareClusterSize);

    for(int n = 0; n<clusters.size(); n++)
    {
        cout << "# "<< n <<": " << clusters[n].indices.size() << endl;
    }

    cout << "Total number of clusters: " << clusters.size () << endl;
    // cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
    // cout << "These are the indices of the points of the initial "
    //     << "cloud that belong to the first cluster:" << endl;
    // int counter = 0;
    // while (counter < clusters[0].indices.size ())
    // {
    //     cout << clusters[0].indices[counter] << ", ";
    //     counter++;
    //     if (counter % 10 == 0)
    //         cout << endl;
    // }
    // cout << endl;

    //==================================//
    // Visualization RegionGrowRGB Result
    //==================================//
    pcl::PointCloud <PointTRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer_region_grow ("Cluster Viewer");
    viewer_region_grow.showCloud(colored_cloud);
    while (!viewer_region_grow.wasStopped())
    {
    }

    //===============//
    // Visualization
    //===============//
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("RegionGrow"));
    pcl::PointCloud<PointTRGB>::Ptr cluster_cloud(new pcl::PointCloud<PointTRGB>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<PointTRGB> extract;
    extract.setInputCloud(scene);
    extract.setNegative(false);
    
    for(int n=0; n<clusters.size(); n++)
    {
        inliers->indices = clusters[n].indices;
    
        cluster_cloud->clear();
        extract.setIndices(inliers);
        extract.filter(*cluster_cloud);

        viewer->removeAllPointClouds();
        viewer->addPointCloud<PointTRGB>(cluster_cloud,"cluster_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cluster_cloud");
        viewer->spin();
    }
    return 0;
}