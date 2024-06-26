// CMakeLists.txt
// cmake ..
// 確認 PCL_INCLUDE_DIRS中使用的vtk
// 將CMakeLists中的 set(VTK_DIR "/usr/include;/usr/include/vtk-7.1")
// 更改為電腦vtk路徑

// make
// ./sac_cylinder

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/visualization/common/shapes.h>

//VTK
#include <vtkNew.h>
#include <vtkNamedColors.h>
#include <vtkSmartPointer.h>

#include <vtkActor.h>

#include <vtkTubeFilter.h>
#include <vtkCylinderSource.h>

#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

typedef pcl::PointXYZ PointT;
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
    
    //============//
    // Preprocess
    //============//
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

    cout << "Total number of clusters: " << clusters.size () << endl;

    //======================//
    // Extract PointClouds
    //======================//
    pcl::PointCloud<PointTRGB>::Ptr table_bottle(new pcl::PointCloud<PointTRGB>);
    pcl::PointCloud<PointTRGB>::Ptr table(new pcl::PointCloud<PointTRGB>);
    pcl::PointCloud<PointTRGB>::Ptr bottle(new pcl::PointCloud<PointTRGB>);
    pcl::PointCloud<PointTRGB>::Ptr box(new pcl::PointCloud<PointTRGB>);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::ExtractIndices<PointTRGB> extract;
    extract.setInputCloud(scene);
    extract.setNegative (false);

    inliers->indices = clusters[0].indices;
    extract.setIndices(inliers);
    extract.filter(*table_bottle);

    inliers->indices = clusters[1].indices;
    extract.setIndices(inliers);
    extract.filter(*box);

    //=======================================//
    // SAmple Consensus (SAC) Segmentation
    //=======================================//
    //https://pointclouds.org/documentation/group__sample__consensus.html
    pcl::ModelCoefficients::Ptr table_coeff(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr bottle_coeff(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr box_coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices);
    pcl::PointIndices::Ptr bottle_inliers(new pcl::PointIndices);
    pcl::PointIndices::Ptr box_inliers(new pcl::PointIndices);

    //=======================//
    // table: SACMODEL_PLANE
    //=======================//
    pcl::SACSegmentation<PointTRGB> seg;
    seg.setOptimizeCoefficients(true);      //optional
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(table_bottle);
    seg.segment(*table_inliers, *table_coeff);

    cout << "平面方程式 Ax+By+Cz+D=0: "
		<< table_coeff->values[0] << "x + "
        << table_coeff->values[1] << "y + "
        << table_coeff->values[2] << "z + "
        << table_coeff->values[3] << " = 0\n";

    pcl::ExtractIndices<PointTRGB> extract_plane;
    extract_plane.setInputCloud(table_bottle);
    extract_plane.setNegative (false);
    extract_plane.setIndices(table_inliers);
    extract_plane.filter(*table);

    pcl::PointCloud<PointTRGB>::Ptr table_outliers(new pcl::PointCloud<PointTRGB>);
    extract_plane.setNegative(true);
    extract_plane.filter(*table_outliers);

    //============================//
    // bottle: SACMODEL_CYLINDER
    //============================//
    //https://pointclouds.org/documentation/tutorials/cylinder_segmentation.html#cylinder-segmentation
    //https://answers.ros.org/question/173143/segmentation-of-a-pointcloud-to-find-a-specific-object-a-cup-pcl/
    //https://blog.csdn.net/stanshi/article/details/124773320?spm=1001.2101.3001.6650.2&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-2-124773320-blog-41117053.pc_relevant_multi_platform_whitelistv3&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-2-124773320-blog-41117053.pc_relevant_multi_platform_whitelistv3&utm_relevant_index=3
    //https://blog.csdn.net/qq_44575789/article/details/126225441
    pcl::PointCloud<pcl::Normal>::Ptr table_outlier_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointTRGB>::Ptr kdtree(new pcl::search::KdTree<PointTRGB>);
    pcl::NormalEstimation<PointTRGB, pcl::Normal> ne;
    ne.setInputCloud(table_outliers);
    ne.setSearchMethod(tree);
    // ne.setRadiusSearch(0.03);
    ne.setKSearch(50);
    ne.compute(*table_outlier_normals);

    pcl::SACSegmentationFromNormals<PointTRGB, pcl::Normal> seg_nor;  //創建圓柱體分割物件
    seg_nor.setOptimizeCoefficients(true);              //[optional] 設定是否需對估計的模型進行參數優化
    seg_nor.setModelType(pcl::SACMODEL_CYLINDER);       //設定分割模型為"圓柱體"
    seg_nor.setMethodType (pcl::SAC_RANSAC);            //使用RANSAC方法進行參數估計
    seg_nor.setNormalDistanceWeight (0.1);              //設定表面法向量的權重係數
    seg_nor.setMaxIterations (10000);                   //設定最大迭代次數
    seg_nor.setDistanceThreshold (0.05);                //設定內點到模型距離的最大值
    seg_nor.setRadiusLimits (0, 0.1);                   //設定圓柱體模型半徑範圍

    seg_nor.setInputCloud(table_outliers);                    //設定輸入點雲
    seg_nor.setInputNormals (table_outlier_normals);          //設定輸入法向量
    seg_nor.segment(*bottle_inliers, *bottle_coeff);    //執行分割

    cout << "圓柱體係數 Cylinder coeff: " //<< *bottle_coeff << endl;
        << "\n\tPoint_on_axis.x  = " << bottle_coeff->values[0] //圓柱體軸線上的一點座標
        << "\n\tPoint_on_axis.y  = " << bottle_coeff->values[1]
        << "\n\tPoint_on_axis.z  = " << bottle_coeff->values[2]
        << "\n\tAxis_direction.x = " << bottle_coeff->values[3] //圓柱體軸線的方向向量
        << "\n\tAxis_direction.y = " << bottle_coeff->values[4]
        << "\n\tAxis_direction.z = " << bottle_coeff->values[5]
        << "\n\tCylinder Radius  = " << bottle_coeff->values[6] //圓柱體半徑
        << endl;

    pcl::ExtractIndices<PointTRGB> extract_bottle;
    extract_bottle.setInputCloud(table_outliers);
    extract_bottle.setNegative (false);         //false:提取圓柱體內點; true:提取圓柱體外點
    extract_bottle.setIndices(bottle_inliers);
    extract_bottle.filter(*bottle);
    cout << "bottle_inliers:" << bottle_inliers->indices.size() << endl;

    // pcl::PointCloud<PointTRGB>::Ptr outliers(new pcl::PointCloud<PointTRGB>);
    // extract_bottle.setNegative(true);
    // extract_bottle.filter(*outliers);

    //========================//
    // 圓柱體的點都投影到軸線上
    //========================//
    //https://www.csdn.net/tags/NtjaYgxsNjY4NTYtYmxvZwO0O0OO0O0O.html
    pcl::PointCloud<PointT>::Ptr proj_axis_cloud(new pcl::PointCloud<PointT>);
    PointT axis_pt = PointT(bottle_coeff->values[0], bottle_coeff->values[1], bottle_coeff->values[2]);
    PointT axis_vect = PointT(bottle_coeff->values[3], bottle_coeff->values[4], bottle_coeff->values[5]);
    for(auto&& pt: *bottle)
    {
        double t = -(axis_vect.x * (axis_pt.x - pt.x) +
                     axis_vect.y * (axis_pt.y - pt.y) +
                     axis_vect.z * (axis_pt.z - pt.z))/
                     (axis_vect.x*axis_vect.x + axis_vect.y*axis_vect.y + axis_vect.z*axis_vect.z);
        PointT proj_pt = PointT(axis_pt.x + axis_vect.x*t, axis_pt.y + axis_vect.y*t, axis_pt.z + axis_vect.z*t); 
        proj_axis_cloud->push_back(proj_pt);
    }

    pcl::visualization::PCLVisualizer::Ptr viewerCylinder(new pcl::visualization::PCLVisualizer("Cylinder proj to Axis"));
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud41_color(proj_axis_cloud, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointTRGB> cloud51_color(bottle, 255, 0, 255);

    viewerCylinder->addPointCloud<PointTRGB>(bottle, cloud51_color, "bottle");
    viewerCylinder->addPointCloud<PointT>(proj_axis_cloud, cloud41_color, "proj_axis_cloud");
    viewerCylinder->spin();

    //===========================//
    // 求圓柱體的高(找最大點、最小點)
    //===========================//
    // 旋轉軸線貼合到X軸:先繞z軸轉，再繞y軸轉
    pcl::PointCloud<PointT>::Ptr proj_axis_cloud_rotate(new pcl::PointCloud<PointT>);

    double angle_z = atan2(axis_vect.y, axis_vect.x);    //axis_vect繞Z軸角度
    double angle_y = atan2(axis_vect.z, axis_vect.x);    //axis_vect繞Y軸角度

    pcl::transformPointCloud(*proj_axis_cloud, *proj_axis_cloud_rotate, Eigen::Affine3f(Eigen::AngleAxisf(angle_z, Eigen::Vector3f::UnitZ())));
    pcl::transformPointCloud(*proj_axis_cloud_rotate, *proj_axis_cloud_rotate, Eigen::Affine3f(Eigen::AngleAxisf(angle_y, Eigen::Vector3f::UnitY())));

    // 找x最大值、最小值的點
    auto&& max_x_pt = std::max_element(proj_axis_cloud_rotate->begin(), proj_axis_cloud_rotate->end(), [](const PointT& a, const PointT&b)
       {return a.x < b.x;}).operator->();

    auto&& min_x_pt = std::min_element(proj_axis_cloud_rotate->begin(), proj_axis_cloud_rotate->end(), [](const PointT& a, const PointT&b)
       {return a.x < b.x;}).operator->();

    // 將x最大值、最小值的點旋轉回原始位置
    pcl::PointCloud<PointT>::Ptr pt_rotate_back(new pcl::PointCloud<PointT>);
    pt_rotate_back->push_back(*max_x_pt);
    pt_rotate_back->push_back(*min_x_pt);
    pcl::transformPointCloud(*pt_rotate_back, *pt_rotate_back, Eigen::Affine3f(Eigen::AngleAxisf(-angle_y, Eigen::Vector3f::UnitY())));
    pcl::transformPointCloud(*pt_rotate_back, *pt_rotate_back, Eigen::Affine3f(Eigen::AngleAxisf(-angle_z, Eigen::Vector3f::UnitZ())));

    PointT start_pt = pt_rotate_back->at(0);
    PointT end_pt = pt_rotate_back->at(1);
    auto&& center_pt = PointT((start_pt.x + end_pt.x)/2.0, (start_pt.y + end_pt.y)/2.0, (start_pt.z + end_pt.z)/2.0);
    cout << start_pt.x <<", "<< start_pt.y <<", "<< start_pt.z << ", " 
         << end_pt.x   <<", "<< end_pt.y   <<", "<< end_pt.z   << ", " 
         << center_pt.x <<", "<< center_pt.y <<", "<< center_pt.z << endl;

   
    //========================//
    // Visualization Cylinder
    //========================//
    vtkNew<vtkNamedColors> colors;

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    
    renderWindow->SetWindowName("Cylinder");
    renderWindow->AddRenderer(renderer);
    
    //=============== Cylinder Point Cloud ================//
    vtkSmartPointer<vtkPoints> vtk_pts = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> vtk_vertices = vtkSmartPointer<vtkCellArray>::New();
    for(int n = 0; n<bottle->size(); n++)
    {
        vtkIdType pid[1];
        pid[0] = vtk_pts->InsertNextPoint(bottle->at(n).x, bottle->at(n).y, bottle->at(n).z);
        vtk_vertices->InsertNextCell(1, pid);
    }

    vtkSmartPointer<vtkPolyData> polydata_cloud = vtkSmartPointer<vtkPolyData>::New();
    polydata_cloud->SetPoints(vtk_pts);
    polydata_cloud->SetVerts(vtk_vertices);

    vtkSmartPointer<vtkPolyDataMapper> mapper_cloud = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_cloud->SetInputData(polydata_cloud);

    vtkSmartPointer<vtkActor> actor_cloud = vtkSmartPointer<vtkActor>::New();
    actor_cloud->SetMapper(mapper_cloud);
    actor_cloud->GetProperty()->SetColor(0.0, 1.0, 0.0);
    actor_cloud->GetProperty()->SetPointSize(3);


    //=============== Cylinder Model ================//
    vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
    lineSource->SetPoint1(start_pt.x, start_pt.y, start_pt.z);
    lineSource->SetPoint2(end_pt.x, end_pt.y, end_pt.z);

    vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
    tubeFilter->SetInputConnection(lineSource->GetOutputPort());
    tubeFilter->SetRadius(bottle_coeff->values[6]);
    tubeFilter->SetNumberOfSides(20);
    tubeFilter->CappingOn();
    tubeFilter->Update();

    vtkSmartPointer<vtkPolyData> polydata = tubeFilter->GetOutput();
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polydata);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(255, 0, 0); //GBR
    actor->GetProperty()->SetEdgeVisibility(1);
    actor->GetProperty()->SetEdgeColor(0.9, 0.9, 0.4);
    // actor->GetProperty()->SetLineWidth(6);
    // actor->GetProperty()->SetPointSize(12);
    // actor->GetProperty()->SetRenderLinesAsTubes(1);
    // actor->GetProperty()->SetRenderPointsAsSpheres(1);
    // // actor->GetProperty()->SetVertexVisibility(1);
    // // actor->GetProperty()->SetVertexColor(0.5,1.0,0.8);

    // Add actor to the scene
    renderer->AddActor(actor);
    renderer->AddActor(actor_cloud);
    renderer->SetBackground(colors->GetColor3d("Black").GetData()); //DarkGreen

    // Render and Interact
    vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
    renderWindowInteractor->SetRenderWindow(renderWindow);

    renderWindow->Render();
    renderWindowInteractor->Start();

    // //================================//
    // // Visualization Cylinder Example
    // //================================//
    // // 顯示自行創件的圓柱體
    // vtkNew<vtkNamedColors> colors;

    // // Create a sphere
    // vtkNew<vtkCylinderSource> cylinderSource;
    // cylinderSource->SetCenter(0.0, 0.0, 0.0);
    // cylinderSource->SetRadius(5.0);
    // cylinderSource->SetHeight(7.0);
    // cylinderSource->SetResolution(100);

    // // Create a mapper and actor
    // vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    // mapper->SetInputConnection(cylinderSource->GetOutputPort());
    
    // vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    // actor->GetProperty()->SetColor(colors->GetColor3d("Cornsilk").GetData());
    // actor->SetMapper(mapper);

    // // Create a renderer, render window, and interactor
    // vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    // vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    // renderWindow->SetWindowName("Cylinder");
    // renderWindow->AddRenderer(renderer);

    // vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
    // renderWindowInteractor->SetRenderWindow(renderWindow);

    // // Add actor to the scene
    // renderer->AddActor(actor);
    // renderer->SetBackground(colors->GetColor3d("DarkGreen").GetData());

    // // Render and Interact
    // renderWindow->Render();
    // renderWindowInteractor->Start();

    return 0;
}