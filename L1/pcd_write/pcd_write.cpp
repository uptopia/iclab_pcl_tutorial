//https://pcl.readthedocs.io/projects/tutorials/en/master/using_pcl_pcl_config.html#using-pcl-pcl-config
//https://pcl.readthedocs.io/projects/tutorials/en/master/writing_pcd.html#writing-pcd
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    //========================//
    // Generate Random Points 
    // for PointCloud
    //========================//
    //https://www.runoob.com/w3cnote/cpp-rand-srand.html
    //https://www.geeksforgeeks.org/rand-and-srand-in-ccpp/
    
    // Set Seed for rand()
    srand(0);               //使用固定的亂數種子，產生相同的亂數結果
    //srand(time(NULL));    //使用時間生成變動的亂數種子，產生不同的亂數結果

    cloud->width = 5;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->resize(cloud->width*cloud->height);

    // Method1
    //*Note: 
    // rand():                      0 ~ RAND_MAX
    // rand()/(RAND_MAX + 1.0f):    0 ~ 1
    // N*rand():                   -RAND_MAX ~ RAND_MAX (數值溢出變成負值)
    // N*rand()/(RAND_MAX + 1.0f): -1 ~ 1
    for(int i=0; i<cloud->width*cloud->height; i++)
    {
        pcl::PointXYZ pt;
        pt.x = 1024*rand()/(RAND_MAX + 1.0f);
        pt.y = 1024*rand()/(RAND_MAX + 1.0f);
        pt.z = 1024*rand()/(RAND_MAX + 1.0f); 
        //rand()/(RAND_MAX + 1.0f)          : 0 ~ 1
        //2*rand()/(RAND_MAX + 1.0f)        :-1 ~ 1
        //2+2*rand()/(RAND_MAX + 1.0f)      : 1 ~ 3
        //2+2*(rand()/(RAND_MAX + 1.0f))    : 2 ~ 4
        cloud->points[i] = pt;
    }

    // //Method2
    // for(auto& point:cloud)
    // {
    //     pointt.x = 1024*rand()/(RAND_MAX + 1.0f);
    //     point.y = 1024*rand()/(RAND_MAX + 1.0f);
    //     point.z = 1024*rand()/(RAND_MAX + 1.0f);
    // }

    //==================//
    // Save PointCloud
    //==================//
    pcl::io::savePCDFileASCII("test_pcd.pcd", *cloud);
    cerr << "Saved" << cloud->size() << "points to test_pcd.pcd" << endl;

    //========================//
    // Print Out Point Values
    //========================//
    // Method1
    for(int i = 0; i< cloud->size(); i++)
    {
        cout << "pt index #" << i << ":" 
            << cloud->points[i].x <<", "
            << cloud->points[i].y <<", "
            << cloud->points[i].z << endl;
    }

    // // Method2
    // for(const auto& point: cloud)
    // {
    //     cerr << "pt index #" << i << ":" 
    //         << cloud->points[i].x <<", "
    //         << cloud->points[i].y <<", "
    //         << cloud->points[i].z << endl;
    // }

    //===============//
    // Visualization
    //===============//
    pcl::visualization::PCLVisualizer::Ptr view(new pcl::visualization::PCLVisualizer("pcd_write"));
    view->removeAllPointClouds();
    view->setBackgroundColor(0, 0, 0);
    view->addCoordinateSystem(1.0f);
    view->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    view->spin();

    return(0);
}