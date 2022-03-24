#include<iostream>
#include<pcl/io/pcd_io.h>

#include<pcl/filters/voxel_grid.h>
#include<pcl/keypoints/uniform_sampling.h>

#include<pcl/filters/passthrough.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include<pcl/filters/radius_outlier_removal.h>
#include<pcl/filters/condition_outlier_removal.h>

using namespace std;
pcl::PointXYZ PointT;

int main()
{
    //===========//
    // Parameters
    //===========//
    file_load_path = "";
    file_save_path = "";
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_dn(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_cleaned(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_removed(new pcl::PointCloud<PointT>);
    //outliers
    //===========//
    // Load PCD
    //===========//     
    if(pcl::io::loadPCDFile<PointT>(file_load_path, *cloud)== -1)
    {
        PCL_ERROR("Failed to load file" + file_load_path);
        system("pause");
        return -1;
    }

    //==============//
    // Downsampling
    //==============//
    //(1) Voxel Grid
    float leafsize = 0.02;
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*cloud_dn);

    //(2) Uniform Sampling
    //https://www.cnblogs.com/li-yao7758258/p/6527969.html 上採樣
    pcl::PointCloud<int> keypoint_indices;
    pcl::UniformSampling<PointT> us;
    us.setInputCloud(cloud);
    us.setRadiusSearch(0.01f);
    us.compute(keypoint_indices);
    pcl::copyPointCloud(*cloud, keypoint_indices.points, *cloud_dn);

    //==============//
    // Filtering
    //==============//
    //https://zhuanlan.zhihu.com/p/95983353
    //(1) PassThrough Filter
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_dn);
    pass.setFilterFieldName('z');
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*cloud_cleaned);

    pass.setFilterLimitsNegative(true);
    pass.filter(*cloud_removed);//outliers

    //(2) Statistical Outlier Removal
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_dn);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_cleaned);

    sor.setNegative(true);
    sor.filter(*cloud_removed); //outliers

    //(3) Radius Outlier Removal
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud_dn);
    ror.setRadiusSearch(0.8);
    ror.setMinNeighborsInRadius(2);
    ror.setKeepOrganized(true);
    ror.filter(*cloud_cleaned);

    //ror.????

    //(4) Conditional Outlier Removal
    // build the condition
    pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT>());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new 
        pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new 
        pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter (*cloud_cleaned);

    //(5) Median Filter (only for ORGANIZED point cloud)
    //https://zhuanlan.zhihu.com/p/95983353

    //===============//
    // Visualization
    //===============//
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->initCameraParameters();
    int v1(0),v2(0);;

    viewer->createViewPort(0.0,0.0,0.5,1.0,v1);//(Xmin,Ymin,Xmax,Ymax)設置窗口座標
    viewer->setBackgroundColor(0,0,0,v1);//設置背景
    viewer->addText("original clouds", 10,10,"v1 text", v1);//設置視口名稱

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud1_color(cloud_cleaned, 0, 255, 0); // green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud2_color(cloud_removed, 255, 0, 0); // green
    
    viewer->addPointCloud<pcl::PointXYZ>(cloud_cleaned, cloud1_color, "cloud1", v1);//添加點雲
    viewer->addPointCloud<pcl::PointXYZ>(cloud_removed, cloud2_color, "cloud2", v1);//添加點雲
	
    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
    viewer->setBackgroundColor(0.3,0.3,0.3,v2);
    viewer->addText("combined cloud", 10,10,"v2 text", v2);
    viewer->addPointCloud(cloud_dn,"cloud_combined", v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE,3,"cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE,3,"cloud2");
    while (!viewer->wasStopped ())
    {

     viewer->spinOnce();
    }

    return 0;
}