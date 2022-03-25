// CMakeLists.txt
// cmake ..
// make
// ./region_grow

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

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/visualization/common/shapes.h>
typedef pcl::PointXYZRGB PointTRGB;

using namespace std;

int main()
{
    std::string file_path = "/home/upup/iclab_pcl_tutorial/L4/sample_scene/scene_organized_cloud.pcd";  
    
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

    // ================//
    // Region Grow RGB
    // ================//
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

    // std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    // std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
    // std::cout << "These are the indices of the points of the initial" <<
    //     std::endl << "cloud that belong to the first cluster:" << std::endl;
    // int counter = 0;
    // while (counter < clusters[0].indices.size ())
    // {
    //     std::cout << clusters[0].indices[counter] << ", ";
    //     counter++;
    //     if (counter % 10 == 0)
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl;

    // ====================//
    // Extract PointClouds
    // ====================//
    pcl::PointCloud<PointTRGB>::Ptr table_bottle(new pcl::PointCloud<PointTRGB>);
    pcl::PointCloud<PointTRGB>::Ptr table(new pcl::PointCloud<PointTRGB>);
    pcl::PointCloud<PointTRGB>::Ptr bottle(new pcl::PointCloud<PointTRGB>);
    pcl::PointCloud<PointTRGB>::Ptr ball(new pcl::PointCloud<PointTRGB>);
    pcl::PointCloud<PointTRGB>::Ptr box(new pcl::PointCloud<PointTRGB>);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::ExtractIndices<PointTRGB> extract;
    extract.setInputCloud(scene);
    extract.setNegative (false);

    inliers->indices = clusters[3].indices;
    extract.setIndices(inliers);
    extract.filter(*table_bottle);

    inliers->indices = clusters[1].indices;
    extract.setIndices(inliers);
    extract.filter(*ball);

    inliers->indices = clusters[2].indices;
    extract.setIndices(inliers);
    extract.filter(*box);

    // ====================================//
    // SAmple Consensus (SAC) Segmentation
    // ====================================//
    //https://pointclouds.org/documentation/group__sample__consensus.html
    pcl::ModelCoefficients::Ptr table_coeff(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr bottle_coeff(new pcl::ModelCoefficients);
    // pcl::ModelCoefficients::Ptr ball_coeff(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr box_coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices);
    pcl::PointIndices::Ptr bottle_inliers(new pcl::PointIndices);
    // pcl::PointIndices::Ptr ball_inliers(new pcl::PointIndices);
    pcl::PointIndices::Ptr box_inliers(new pcl::PointIndices);

    //table: SACMODEL_PLANE
    pcl::SACSegmentation<PointTRGB> seg;
    seg.setOptimizeCoefficients(true); //optional
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(table_bottle);
    seg.segment(*table_inliers, *table_coeff);

    pcl::ExtractIndices<PointTRGB> extract_plane;
    extract_plane.setInputCloud(table_bottle);
    extract_plane.setNegative (false);
    extract_plane.setIndices(table_inliers);
    extract_plane.filter(*table);
    extract_plane.setNegative(true);
    pcl::PointCloud<PointTRGB>::Ptr outliers(new pcl::PointCloud<PointTRGB>);
    extract_plane.filter(*outliers);


    //bottle: SACMODEL_CYLINDER
    //https://pointclouds.org/documentation/tutorials/cylinder_segmentation.html#cylinder-segmentation
    //https://answers.ros.org/question/173143/segmentation-of-a-pointcloud-to-find-a-specific-object-a-cup-pcl/
    pcl::PointCloud<pcl::Normal>::Ptr outlier_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointTRGB>::Ptr kdtree(new pcl::search::KdTree<PointTRGB>);
    pcl::NormalEstimation<PointTRGB, pcl::Normal> ne;
    ne.setInputCloud(outliers);
    ne.setSearchMethod(tree);
    // ne.setRadiusSearch(0.03);
    ne.setKSearch(50);
    ne.compute(*outlier_normals);

    pcl::SACSegmentationFromNormals<PointTRGB, pcl::Normal> seg_nor;
    // pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    seg_nor.setOptimizeCoefficients(true); //optional
    seg_nor.setModelType(pcl::SACMODEL_CYLINDER);
    seg_nor.setMethodType (pcl::SAC_RANSAC);
    seg_nor.setNormalDistanceWeight (0.1);
    seg_nor.setMaxIterations (10000);
    seg_nor.setDistanceThreshold (0.05);
    seg_nor.setRadiusLimits (0, 0.1);
    seg_nor.setInputNormals (outlier_normals);
    seg_nor.setInputCloud(outliers);
    seg_nor.segment(*bottle_inliers, *bottle_coeff);
    std::cerr << "Cylinder coefficients: " << *bottle_coeff<< std::endl;


    pcl::ExtractIndices<PointTRGB> extract_bottle;
    extract_bottle.setInputCloud(outliers);
    extract_bottle.setNegative (false);
    extract_bottle.setIndices(bottle_inliers);
    extract_bottle.filter(*bottle);
    cout<<"bottle_inliers:"<<bottle_inliers->indices.size() <<endl;
    // extract_bottle.setNegative(true);
    // pcl::PointCloud<PointTRGB>::Ptr outliers(new pcl::PointCloud<PointTRGB>);
    // extract_bottle.filter(*outliers);

    //ball: SACMODEL_SPHERE
    //https://blog.csdn.net/weixin_46098577/article/details/121977744
    pcl::SampleConsensusModelSphere<PointTRGB>::Ptr model_sphere(new pcl::SampleConsensusModelSphere<PointTRGB>(ball));	//选择拟合点云与几何模型
	pcl::RandomSampleConsensus<PointTRGB> ransac(model_sphere);	//创建随机采样一致性对象
	ransac.setDistanceThreshold(0.005);	//设置距离阈值，与球面距离小于0.01的点作为内点
	ransac.computeModel();				//执行模型估计

    pcl::PointCloud<PointTRGB>::Ptr ball_clean(new pcl::PointCloud<PointTRGB>);
    std::vector<int> ball_inliers;
    ransac.getInliers(ball_inliers);
    pcl::copyPointCloud<PointTRGB>(*ball, ball_inliers, *ball_clean);

    Eigen::VectorXf ball_coeff;
	ransac.getModelCoefficients(ball_coeff);
	cout << "->球面方程为：\n"
		<< "(x - " << ball_coeff[0]
		<< ") ^ 2 + (y - " << ball_coeff[1]
		<< ") ^ 2 + (z - " << ball_coeff[2]
		<< ")^2 = " << ball_coeff[3]
		<< " ^2"
		<< endl;

    // seg.setOptimizeCoefficients(true); //optional
    // seg.setModelType(pcl::SACMODEL_SPHERE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setDistanceThreshold(0.005);
    // seg.setInputCloud(ball);
    // seg.segment(*ball_inliers, *ball_coeff);

    // pcl::ExtractIndices<PointTRGB> extract_ball;
    // extract_plane.setInputCloud(ball);
    // extract_plane.setNegative (false);
    // extract_plane.setIndices(ball_inliers);
    // extract_plane.filter(*ball);
    // // extract_plane.setNegative(true);
    // // pcl::PointCloud<PointTRGB>::Ptr outliers(new pcl::PointCloud<PointTRGB>);
    // // extract_plane.filter(*outliers);

    //box

    // ===============//
    // Visualization
    // ===============//
    // pcl::PointCloud <PointTRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    // pcl::visualization::CloudViewer viewer ("Cluster viewer");
    // viewer.showCloud(colored_cloud);
    // while (!viewer.wasStopped ())
    // {
    // }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("RegionGrow"));
    pcl::visualization::PointCloudColorHandlerCustom<PointTRGB> cloud1_color(table, 255, 0, 0); // green
    pcl::visualization::PointCloudColorHandlerCustom<PointTRGB> cloud2_color(ball, 0, 255, 0); // green
    pcl::visualization::PointCloudColorHandlerCustom<PointTRGB> cloud3_color(box, 0, 255, 255); // green
    pcl::visualization::PointCloudColorHandlerCustom<PointTRGB> cloud4_color(outliers, 255, 255, 255); // green
    pcl::visualization::PointCloudColorHandlerCustom<PointTRGB> cloud5_color(bottle, 255, 0, 255); // green

    viewer->addPointCloud<PointTRGB>(bottle, cloud5_color, "cloud5");//添加點雲
    // viewer->addPointCloud<PointTRGB>(outliers, cloud4_color, "cloud4");//添加點雲
    viewer->addPointCloud<PointTRGB>(table, cloud1_color, "cloud1");//添加點雲
    // viewer->addPointCloud<PointTRGB>(ball, cloud2_color, "cloud2");//添加點雲
    viewer->addPointCloud<PointTRGB>(ball_clean, cloud2_color, "cloud2");//添加點雲
    // viewer->addPointCloud<PointTRGB>(box, cloud3_color, "cloud3");//添加點雲

    // pcl::visualization::createCylinder(*bottle_coeff,30);

    // pcl::ModelCoefficients::Ptr ball_coeff_tmp(new pcl::ModelCoefficients);
    // ball_coeff_tmp->values.resize(4);
    // ball_coeff_tmp->values[0] = ball_coeff[0];
    // ball_coeff_tmp->values[1] = ball_coeff[1];
    // ball_coeff_tmp->values[2] = ball_coeff[2];
    // ball_coeff_tmp->values[3] = ball_coeff[3];
    // pcl::visualization::createSphere(*ball_coeff_tmp,10);

    
    pcl::PointXYZ center;
    center.x = ball_coeff[0];
    center.y = ball_coeff[1];
    center.z = ball_coeff[2];
    viewer->addSphere(center, ball_coeff[3], "sphere");
    viewer->addCylinder(*bottle_coeff, "cylinder");//添加指定高度和切片数的圆柱https://blog.csdn.net/l_h2010/article/details/41117053
    viewer->addPlane(*table_coeff, "table_coeff"); //https://github.com/Marcus-Davi/Cpp-PCL/blob/243b6d494d424677dfa3319ab2bb37a7d5572e21/src/plane.cpp

    // for(int k=0; k<clusters.size(); k++)
    // {
    //     pcl::PointCloud<PointTRGB>::Ptr part(new pcl::PointCloud<PointTRGB>);
    //     pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    //     indices->indices = clusters[k].indices;

    //     //https://pointclouds.org/documentation/classpcl_1_1_extract_indices_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html
    //     pcl::ExtractIndices<PointTRGB> extract;
    //     extract.setInputCloud(scene);
    //     extract.setIndices(indices);
    //     extract.setNegative (false);
    //     extract.filter(*part);

    //     pcl::visualization::PointCloudColorHandlerCustom<PointTRGB> cloud_color(part, 255-k*255/clusters.size(),100,50+255/clusters.size());
    //     std::string cloud_name = "cloud_name"+std::to_string(k);
    //     viewer->addPointCloud<PointTRGB>(part, cloud_color, cloud_name);
    // }
    viewer->spin();

    return 0;
}