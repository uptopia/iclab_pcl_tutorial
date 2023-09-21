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
reg.setMinClusterSize(600); //若cluster點數少於設定值，則與其他cluster合併

std::vector <pcl::PointIndices> clusters;
reg.extract(clusters);

//===================//
// Sort Cluster Size
//===================//
bool compareClusterSize(pcl::PointIndices& c1, pcl::PointIndices& c2)
{
return c1.indices.size() > c2.indices.size();
}

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

//=======================================//
// SAmple Consensus (SAC) Segmentation
//=======================================//
//https://pointclouds.org/documentation/group__sample__consensus.html



//=======================//
// table: SACMODEL_PLANE
//=======================//
pcl::ModelCoefficients::Ptr table_coeff(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices);

pcl::SACSegmentation<PointTRGB> seg;
seg.setOptimizeCoefficients(true);  //optional
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
pcl::ModelCoefficients::Ptr bottle_coeff(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr bottle_inliers(new pcl::PointIndices);

pcl::PointCloud<pcl::Normal>::Ptr table_outlier_normals (new pcl::PointCloud<pcl::Normal>);
pcl::search::KdTree<PointTRGB>::Ptr kdtree(new pcl::search::KdTree<PointTRGB>);
pcl::NormalEstimation<PointTRGB, pcl::Normal> ne;
ne.setInputCloud(table_outliers);
ne.setSearchMethod(tree);
// ne.setRadiusSearch(0.03);
ne.setKSearch(50);
ne.compute(*table_outlier_normals);

pcl::SACSegmentationFromNormals<PointTRGB, pcl::Normal> seg_nor;  //創建圓柱體分割物件
seg_nor.setOptimizeCoefficients(true);          //[optional] 設定是否需對估計的模型進行參數優化
seg_nor.setModelType(pcl::SACMODEL_CYLINDER);   //設定分割模型為"圓柱體"
seg_nor.setMethodType (pcl::SAC_RANSAC);        //使用RANSAC方法進行參數估計
seg_nor.setNormalDistanceWeight (0.1);          //設定表面法向量的權重係數
seg_nor.setMaxIterations (10000);               //設定最大迭代次數
seg_nor.setDistanceThreshold (0.05);            //設定內點到模型距離的最大值
seg_nor.setRadiusLimits (0, 0.1);               //設定圓柱體模型半徑範圍

seg_nor.setInputCloud(table_outliers);//設定輸入點雲
seg_nor.setInputNormals (table_outlier_normals);  //設定輸入法向量
seg_nor.segment(*bottle_inliers, *bottle_coeff);//執行分割

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
extract_bottle.setNegative (false); //false:提取圓柱體內點; true:提取圓柱體外點
extract_bottle.setIndices(bottle_inliers);
extract_bottle.filter(*bottle);
cout << "bottle_inliers:" << bottle_inliers->indices.size() << endl;

// pcl::PointCloud<PointTRGB>::Ptr outliers(new pcl::PointCloud<PointTRGB>);
// extract_bottle.setNegative(true);
// extract_bottle.filter(*outliers);

//=======================//
// ball: SACMODEL_SPHERE
//=======================//
pcl::SampleConsensusModelSphere<PointTRGB>::Ptr model_sphere(new pcl::SampleConsensusModelSphere<PointTRGB>(ball));
pcl::RandomSampleConsensus<PointTRGB> ransac(model_sphere);
ransac.setDistanceThreshold(0.005);	
ransac.computeModel();				

pcl::PointCloud<PointTRGB>::Ptr ball_clean(new pcl::PointCloud<PointTRGB>);
std::vector<int> ball_inliers;
ransac.getInliers(ball_inliers);
pcl::copyPointCloud<PointTRGB>(*ball, ball_inliers, *ball_clean);

Eigen::VectorXf ball_coeff;
ransac.getModelCoefficients(ball_coeff);
cout << "球面方程式："
	<< "(x - " << ball_coeff[0]
	<< ")^2 + (y - " << ball_coeff[1]
	<< ")^2 + (z - " << ball_coeff[2]
	<< ")^2 = " << ball_coeff[3]
	<< "^2\n";