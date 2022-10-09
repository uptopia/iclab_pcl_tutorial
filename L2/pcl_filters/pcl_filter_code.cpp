#include<iostream>
#include<pcl/io/pcd_io.h>

#include<pcl/filters/voxel_grid.h>

#include<pcl/filters/passthrough.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include<pcl/filters/radius_outlier_removal.h>
#include<pcl/filters/conditional_removal.h>

#include<pcl/visualization/pcl_visualizer.h>

using namespace std;
typedef pcl::PointXYZ PointT;

void show_result(std::string window_name, pcl::PointCloud<PointT>::Ptr& cloud, pcl::PointCloud<PointT>::Ptr& cloud_filtered)
{
    //===============//
    // Visualization
    //===============//
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(window_name));
    viewer->initCameraParameters();
    int v1(0),v2(0);

    // viewport 1: ori + voxel
    viewer->createViewPort(0.0,0.0,0.5,1.0,v1); //(Xmin,Ymin,Xmax,Ymax)設置窗口座標
    viewer->setBackgroundColor(0,0,0,v1);       //設置背景
    viewer->addText("original clouds (rd); filtered cloud (wh)", 10,10,"v1 text", v1);
    viewer->addPointCloud<PointT>(cloud, pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud, 255,0,0), "cloud1", v1);
    viewer->addPointCloud<PointT>(cloud_filtered, pcl::visualization::PointCloudColorHandlerCustom<PointT>(cloud_filtered, 255, 255, 255), "cloud_filtered", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"cloud_filtered");

    //viewport 2: voxel
    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);
    viewer->setBackgroundColor(0.3,0.3,0.3,v2);
    viewer->addText("filtered cloud, total pts = " + to_string(cloud_filtered->size()), 10,10,"cloud_filtered text", v2);
    viewer->addPointCloud(cloud_filtered,"cloud_filtered_only", v2);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce();
    }
}

int main()
{
    //===========//
    // Parameters
    //===========//
    std::string file_load_path = "../../../example_data/scene_D435i.pcd";

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_dn(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_passthrough(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_conditioned(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_cleaned(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_removed(new pcl::PointCloud<PointT>);

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
    float leafsize = 0.005;
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*cloud_dn);
  
    //===================//
    // Filtering by XYZ
    //===================//
    //https://zhuanlan.zhihu.com/p/95983353
    //(1) PassThrough Filter 
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_dn);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.24, 0.2);
    pass.filter(*cloud_passthrough);

    pass.setInputCloud(cloud_passthrough);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.13, 0.174);
    pass.filter(*cloud_passthrough);

    pass.setInputCloud(cloud_passthrough);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.64, -0.52);
    pass.filter(*cloud_passthrough);

    pass.setFilterLimitsNegative(true);
    pass.filter(*cloud_removed);//outliers
    show_result(std::string("passthrough filter"),cloud_dn, cloud_passthrough);

    //(2) Conditional Outlier Removal
    // build the condition
    pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT>());
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GT, -0.24)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, 0.2)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, -0.13)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, 0.174)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, -0.64)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, -0.52)));

    pcl::ConditionalRemoval<PointT> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud_dn);
    condrem.setKeepOrganized(true);
    condrem.filter (*cloud_conditioned);
    show_result(std::string("conditional outlier removal filter"),cloud_dn, cloud_conditioned);

    //==========================//
    // Filtering by Space Plane
    //==========================//
    //Space plane equation: ax+by+cz=d
    pcl::PointCloud<PointT>::Ptr cloud_above(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_below(new pcl::PointCloud<PointT>);
    
    pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
    //just an example to randomly generate a plane 
    plane_coeff->values.resize(4);
    plane_coeff->values[0] = 2.3;
    plane_coeff->values[1] = 4.5;
    plane_coeff->values[2] = 1;
    PointT tmp = cloud_dn->points[5000];
    plane_coeff->values[3] = plane_coeff->values[0]*tmp.x +plane_coeff->values[1]*tmp.y +plane_coeff->values[2]*tmp.z;

    for(int n = 0; n < cloud_dn->size(); n++)
    {
        PointT pt = cloud_dn->points[n];
        float val1 = plane_coeff->values[0]*pt.x +plane_coeff->values[1]*pt.y +plane_coeff->values[2]*pt.z;
        float val2 = plane_coeff->values[3];
        if(val1 > val2)
            cloud_above->push_back(pt);
        else
            cloud_below->push_back(pt);
    }
    cout << "cloud_above points = " << cloud_above->size() << endl;
    cout << "cloud_below points = " << cloud_below->size() << endl;
    show_result(std::string("Space plane filter, Plane: "+ \
        to_string(plane_coeff->values[0])+ "x +" + \
        to_string(plane_coeff->values[1])+ "y +" + \
        to_string(plane_coeff->values[2])+ "z = " + \
        to_string(plane_coeff->values[3])),cloud_dn, cloud_above);

    //===============================================//
    // Filtering by Points' neighbor characteristics
    //===============================================//
    //(1) Statistical Outlier Removal
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_passthrough);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_cleaned);

    sor.setNegative(true);
    sor.filter(*cloud_removed); //outliers
    show_result(std::string("statistical outlier removal"), cloud_removed, cloud_cleaned);

    //(2) Radius Outlier Removal
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud_passthrough);
    ror.setRadiusSearch(0.01);
    ror.setMinNeighborsInRadius(10);
    ror.setKeepOrganized(true);
    ror.filter(*cloud_cleaned);

    ror.setNegative(true);
    ror.filter(*cloud_removed); //outliers
    show_result(std::string("radius outlier removal"), cloud_removed, cloud_cleaned);

    return 0;
}