// CMakeLists.txt
// cmake ..
// make
// ./region_grow
//https://stackoverflow.com/questions/16572666/using-clustered-indexes-from-point-cloud-in-rgb-image

#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/passthrough.h>
#include<pcl/search/kdtree.h>
#include<pcl/features/normal_3d.h>
#include<pcl/segmentation/region_growing_rgb.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGB PointTRGB;

using namespace std;

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

    //(2)removeNaNFromPointCloud
    scene->is_dense = false;
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*scene_ori, *scene_ori, nan_indices);

    //(1) Voxel Grid
    float leafsize = 0.005;
    pcl::VoxelGrid<PointTRGB> vg;
    vg.setInputCloud(scene_ori);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*scene);


    pcl::search::Search <PointTRGB>::Ptr tree (new pcl::search::KdTree<PointTRGB>);
    pcl::RegionGrowingRGB<PointTRGB> reg;
    reg.setInputCloud (scene);         // 입력 
    reg.setSearchMethod (tree);        // 탐색 방법 
    reg.setDistanceThreshold (10);     // 10 이웃(Neighbor)으로 지정되는 거리 정보 
    reg.setPointColorThreshold (3);    // 6 동일 Cluter여부를 테스트 하기 위해 사용 (cf. Just as angle threshold is used for testing points normals )
    reg.setRegionColorThreshold (5);   // 5 동일 Cluter여부를 테스트 하기 위해 사용, 통합(merging)단계에서 사용됨 
    reg.setMinClusterSize (600);       // 600 최소 포인트수, 지정 값보다 작으면 이웃 포인트와 통합 됨 

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

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
    // viewer->addPointCloud<PointTRGB>(scene,"scene");
    // viewer->spin();

    pcl::PointCloud <PointTRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped ())
    {
    }

    return 0;
}