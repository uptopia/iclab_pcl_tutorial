#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

typedef pcl::PointXYZRGB PointTRGB;

int main()
{
    std::string file_path = "../../../example_data/scene_D435i.pcd";//organized_cloud_tmp.pcd";
    
    pcl::PointCloud<PointTRGB>::Ptr cloud(new pcl::PointCloud<PointTRGB>);

    pcl::io::loadPCDFile<PointTRGB>(file_path, *cloud);

    //=================//
    // Show RGB Value
    //=================//
    for(int i=0; i<cloud->size(); i++)
    {
        cout << "(rgb, r, g, b) =" 
            << cloud->points[i].rgb << ", "
            << (int)cloud->points[i].r << ", "
            << (int)cloud->points[i].g << ", "
            << (int)cloud->points[i].b << endl;
    }
    
    pcl::visualization::PCLVisualizer::Ptr view(new pcl::visualization::PCLVisualizer("pcd_write"));
    view->removeAllPointClouds();
    view->setBackgroundColor(0, 0, 0);
    view->addCoordinateSystem(1.0f);
    view->addPointCloud<PointTRGB>(cloud, "cloud");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    view->spin();
    

    return 0;
}
