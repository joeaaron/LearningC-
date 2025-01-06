#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/gp3.h>
#include <chrono>

using namespace std;
#define ENABLE_DISPLAY 0		// 定义一个宏，用于控制显示状态

typedef pcl::PointXYZ PointT;

void VisualizedCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr& pSmoothedCloud, const pcl::PointCloud<PointT>::Ptr& pCloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("点云边界提取"));

	int v1(0);
	MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
	MView->addText("Raw cloud", 10, 10, "v1_text", v1);

	int v2(0);
	MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
	MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
	MView->addText("Smoothed cloud", 10, 10, "v2_text", v2);

	MView->addPointCloud<PointT>(pCloud, "Raw cloud", v1);
	MView->addPointCloud<pcl::PointNormal>(pSmoothedCloud, "Smoothed cloud", v2);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Raw cloud", v1);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "Smoothed cloud", v2);
	MView->addCoordinateSystem(1.0);
	MView->initCameraParameters();

	MView->spin();
}

static void CalcNormal(pcl::PointCloud<pcl::Normal>::Ptr& pNormal, const pcl::PointCloud<PointT>::Ptr& pCloud)
{
	// 构建KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(pCloud);

	// 使用NormalEstimation计算法向量
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(pCloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(2); // 设置邻域半径
	ne.setNumberOfThreads(4);

	ne.compute(*pNormal);
}

static void MarchingCubesTriangles(pcl::PolygonMesh& triangles,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals)
{

	//---------初始化MarchingCubes对象，并设置参数-------
	pcl::MarchingCubes<pcl::PointNormal>* mc;
	mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
	//创建搜索树
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	mc->setInputCloud(cloud_with_normals);
	//设置MarchingCubes对象的参数
	mc->setIsoLevel(0.0f);//该方法设置要提取表面的iso级别
	mc->setGridResolution(50, 50, 50);//用于设置行进立方体网格分辨率
	mc->setPercentageExtendGrid(0.0f);//该参数定义在点云的边框和网格限制之间的网格内应该保留多少自由空间
	//------创建多变形网格，用于存储结果------------
	mc->reconstruct(triangles);//执行重构，结果保存在mesh中

#if ENABLE_DISPLAY
	//----------------------------------结果可视化-----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"移动立方体");
	int v1(0), v2(0);
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);	 // 可视化点云
	viewer->addPolygonMesh(triangles, "my", v2);                 // 可视化模型重建结果
	viewer->setRepresentationToSurfaceForAllActors();            // 网格模型以线框图模式显示
	//viewer->addCoordinateSystem(0.2);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
}

void GreedyTriangle(pcl::PolygonMesh& triangles,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals)
{

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//定义搜索树对象
	tree2->setInputCloud(cloud_with_normals);				//利用有向点云构造tree

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(10);								//设置搜索半径radius，来确定三角化时k一邻近的球半径。

	// Set typical values for the parameters
	gp3.setMu(3);							 //设置样本点到最近邻域距离的乘积系数 mu 来获得每个样本点的最大搜索距离，这样使得算法自适应点云密度的变化
	gp3.setMaximumNearestNeighbors(100);	 //设置样本点最多可以搜索的邻域数目100 。
	gp3.setMaximumSurfaceAngle(M_PI / 4);    //45 degrees，设置连接时的最大角度 eps_angle ，当某点法线相对于采样点的法线偏离角度超过该最大角度时，连接时就不考虑该点。
	gp3.setMinimumAngle(M_PI / 5);           //10 degrees，设置三角化后三角形的最小角，参数 minimum_angle 为最小角的值。
	gp3.setMaximumAngle(2 * M_PI / 3);		 //120 degrees，设置三角化后三角形的最大角，参数 maximum_angle 为最大角的值。
	gp3.setNormalConsistency(false);		 //设置一个标志 consistent ，来保证法线朝向一致，如果设置为 true 则会使得算法保持法线方向一致，如果为 false 算法则不会进行法线一致性检查。
	// Get result
	gp3.setInputCloud(cloud_with_normals);	 //设置输入点云为有向点云
	gp3.setSearchMethod(tree2);				 //设置搜索方式tree2
	gp3.reconstruct(triangles);				 //重建提取三角化

	// std::cout << triangles;
	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();//获得重建后每点的 ID, Parts 从 0 开始编号， a-1 表示未连接的点。
	/*
	获得重建后每点的状态，取值为 FREE 、 FRINGE 、 BOUNDARY 、 COMPLETED 、 NONE 常量，
	其中 NONE 表示未定义，
	FREE 表示该点没有在三角化后的拓扑内，为自由点，
	COMPLETED 表示该点在三角化后的拓扑内，并且邻域都有拓扑点，
	BOUNDARY 表示该点在三角化后的拓扑边缘，
	FRINGE 表示该点在三角化后的拓扑内，其连接会产生重叠边。
	*/
	std::vector<int> states = gp3.getPointStates();

#if ENABLE_DISPLAY
	//----------------------------------结果可视化-----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"贪婪投影三角化");
	//int v1(0), v2(0);
	//viewer->createViewPort(0, 0, 0.5, 1, v1);
	////viewer->createViewPort(0.5, 0, 1, 1, v2);
	//viewer->setBackgroundColor(0, 0, 0, v1);
	//viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");	 // 可视化点云
	viewer->addPolygonMesh(triangles, "my");                 // 可视化模型重建结果
	viewer->setRepresentationToSurfaceForAllActors();            // 网格模型以线框图模式显示
	//viewer->addCoordinateSystem(0.2);
	viewer->initCameraParameters();
	viewer->spin();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
}

// Moving Least Squares（MLS）平滑
void MovingLeastSquares(pcl::PointCloud<PointT>::Ptr& pSmoothedCloud, const pcl::PointCloud<PointT>::Ptr& pCloud)
{	
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls_omp; // OMP版本的MLS
	mls_omp.setInputCloud(pCloud);
	mls_omp.setSearchMethod(pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());  // 使用KdTree加速搜索
	mls_omp.setSearchRadius(6);		    // 设置搜索半径
	//mls_omp.setPolynomialFit(true);   // 使用多项式拟合
	mls_omp.setPolynomialOrder(2);		// 设置多项式的阶数
	//mls_omp.setPolynomialFit(false);  // 设置为false可以 加速 smooth
	mls_omp.setSqrGaussParam(0.01);		// 高斯函数的参数，影响平滑程度
	mls_omp.setNumberOfThreads(4);		// 设置使用的线程数，根据CPU核心数调整
	mls_omp.process(*pSmoothedCloud);
}

// 基于邻域的平均法向平滑算法
void NeighborBasedNormal(pcl::PointCloud<pcl::PointNormal>::Ptr& pSmoothedCloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals, const pcl::PointCloud<PointT>::Ptr& pCloud)
{
	// 法向量计算
	pcl::PointCloud<pcl::Normal>::Ptr pNormal(new pcl::PointCloud<pcl::Normal>);
	CalcNormal(pNormal, pCloud);

	// 将点云与法向量合并
	pcl::concatenateFields(*pCloud, *pNormal, *cloud_with_normals);

	// 计算点云密度
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(pCloud);
	std::vector<float> point_densities(pCloud->points.size(), 0.0f);
	int k = 10;  // 邻域点数
	for (size_t i = 0; i < pCloud->points.size(); ++i) {
		std::vector<int> pointIdxNKNSearch(k);
		std::vector<float> pointNKNSquaredDistance(k);
		if (kdtree.nearestKSearch(pCloud->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
			float average_distance = 0.0f;
			for (size_t j = 1; j < pointIdxNKNSearch.size(); ++j) {  // 跳过第一个点（自身）
				average_distance += std::sqrt(pointNKNSquaredDistance[j]);
			}
			average_distance /= (k - 1);
			point_densities[i] = 1.0f / average_distance;  // 密度 = 距离的倒数
		}
	}

	pcl::KdTreeFLANN<pcl::PointNormal> kdtree_normals;
	kdtree_normals.setInputCloud(cloud_with_normals);

	float alpha = 2.0f;  // 搜索半径缩放系数

	for (size_t i = 0; i < cloud_with_normals->points.size(); ++i) {
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		float radius = alpha / point_densities[i];  // 动态半径
		// 搜索邻域
		if (kdtree_normals.radiusSearch(cloud_with_normals->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			Eigen::Vector3f normal_sum(0.0, 0.0, 0.0);

			// 累加法向量
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
				normal_sum += cloud_with_normals->points[pointIdxRadiusSearch[j]].getNormalVector3fMap();
			}

			// 平均并归一化
			normal_sum.normalize();
			pcl::PointNormal point = cloud_with_normals->points[i];
			point.normal_x = normal_sum[0];
			point.normal_y = normal_sum[1];
			point.normal_z = normal_sum[2];

			pSmoothedCloud->points.push_back(point);
		}
	}

	pSmoothedCloud->width = pSmoothedCloud->points.size();
	pSmoothedCloud->height = 1;
	pSmoothedCloud->is_dense = true;

}

// 计算法向量的标准差
static void ComputeNormalVariance(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals)
{
	// 计算法向量的均值
	Eigen::Vector3f normal_mean(0.0f, 0.0f, 0.0f);
	for (size_t i = 0; i < cloud_with_normals->points.size(); ++i) {
		Eigen::Vector3f normal = cloud_with_normals->points[i].getNormalVector3fMap();
		// 检查法向量是否包含NaN
		if (normal.hasNaN()) {
			//std::cerr << "Warning: NaN detected in normal vector at index " << i << std::endl;
			continue;  // 跳过包含NaN的法向量
		}
		normal_mean += normal;
	}
	normal_mean /= cloud_with_normals->points.size();

	// 计算法向量的方差
	float variance = 0.0f;
	for (size_t i = 0; i < cloud_with_normals->points.size(); ++i) {
		Eigen::Vector3f normal = cloud_with_normals->points[i].getNormalVector3fMap();
		if (normal.hasNaN()) {
			//std::cerr << "Warning: NaN detected in normal vector at index " << i << std::endl;
			continue;  // 跳过包含NaN的法向量
		}
		float diff = (normal - normal_mean).norm();  // 计算法向量与均值的欧氏距离
		variance += diff * diff;
	}

	// 方差
	variance /= cloud_with_normals->points.size();
	// 标准差
	float standard_deviation = std::sqrt(variance);

	// 输出法向量的标准差，作为一致性的指标
	std::cout << "Normal Vector Standard Deviation: " << standard_deviation << std::endl;
}

static int ReadFile(std::vector<Eigen::Vector3d>& vNoramls, std::string filePath)
{
	// 打开文件
	std::ifstream file(filePath);
	if (!file.is_open()) {
		std::cerr << "Unable to open file " << filePath << std::endl;
		return -1;
	}

	std::string line;
	// 逐行读取文件
	while (std::getline(file, line)) {
		std::istringstream ss(line);
		double x, y, z, nx, ny, nz;

		// 读取每一行中的值
		ss >> x >> y >> z >> nx >> ny >> nz;

		// 将后三列的值存入Eigen::Vector3d对象
		Eigen::Vector3d point(nx, ny, nz);
		vNoramls.push_back(point);
	}

	// 关闭文件
	file.close();

	return 0;
	//// 打印提取的值
	//for (const auto& point : vNoramls) {
	//	std::cout << "Vector: " << point.transpose() << std::endl;
	//}

}

//static void CalcNormalVariance(const std::vector<Eigen::Vector3d>& points)
//{
//	// 计算法向量的均值
//	Eigen::Vector3d mean(0, 0, 0);
//	for (const auto& point : points) {
//		mean += point;
//	}
//	mean /= points.size();
//
//	std::cout << "Mean of the normal vectors: " << mean.transpose() << std::endl;
//
//	// 计算法向量的方差
//	//Eigen::Vector3d variance(0, 0, 0);
//	double variance = 0.0;
//	for (const auto& point : points) {
//		double diff = (point - mean).norm();
//		//variance += diff.cwiseProduct(diff);
//		variance += diff * diff;
//	}
//
//	variance /= points.size();
//	std::cout << "Standard deviation of the normal vectors: " << std::sqrt(variance) << std::endl;
//}

static void CalcNormalVariance(const std::vector<Eigen::Vector3d>& points)
{
	// 计算法向量的均值
	Eigen::Vector3d mean(0, 0, 0);
	for (const auto& point : points) {
		mean += point;
	}
	mean /= points.size();

	std::cout << "Mean of the normal vectors: " << mean.transpose() << std::endl;

	// 计算法向量的方差
	double variance = 0.0;
	std::vector<double> angle_differences;  // 用来存储角度差异
	for (size_t i = 0; i < points.size(); ++i) {
		const Eigen::Vector3d& point = points[i];
		double diff = (point - mean).norm();
		variance += diff * diff;

		// 计算相邻法向量之间的夹角差异
		if (i > 0) {
			const Eigen::Vector3d& prev_point = points[i - 1];
			// 计算夹角
			double cos_angle = point.dot(prev_point) / (point.norm() * prev_point.norm());
			// 限制cos值范围在[-1, 1]之间，避免数值误差
			cos_angle = std::clamp(cos_angle, -1.0, 1.0);
			double angle = std::acos(cos_angle);  // 计算夹角（弧度）
			angle_differences.push_back(angle);
		}
	}

	// 法向量方差（标准差）
	variance /= points.size();
	double normal_standard_deviation = std::sqrt(variance);

	// 计算角度差异的标准差
	double angle_variance = 0.0;
	for (double angle : angle_differences) {
		angle_variance += angle * angle;
	}
	angle_variance /= angle_differences.size();
	double angle_standard_deviation = std::sqrt(angle_variance);

	// 输出法向量的标准差和法向量夹角的标准差
	std::cout << "Standard Deviation of the normal vectors: " << normal_standard_deviation << std::endl;
	std::cout << "Standard Deviation of the normal vector angles: " << angle_standard_deviation << " radians" << std::endl;
}

static void ComputeNormalVarianceAndAngle(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals)
{
	// 计算法向量的均值
	Eigen::Vector3f normal_mean(0.0f, 0.0f, 0.0f);
	for (size_t i = 0; i < cloud_with_normals->points.size(); ++i) {
		Eigen::Vector3f normal = cloud_with_normals->points[i].getNormalVector3fMap();
		if (normal.hasNaN()) {
			continue;  // 跳过包含NaN的法向量
		}
		normal_mean += normal;
	}
	normal_mean /= cloud_with_normals->points.size();

	// 计算法向量的方差
	float variance = 0.0f;
	std::vector<float> angle_differences;
	for (size_t i = 0; i < cloud_with_normals->points.size(); ++i) {
		Eigen::Vector3f normal = cloud_with_normals->points[i].getNormalVector3fMap();
		if (normal.hasNaN()) {
			continue;  // 跳过包含NaN的法向量
		}
		float diff = (normal - normal_mean).norm();  // 计算法向量与均值的欧氏距离
		variance += diff * diff;

		// 计算法向量与前一个法向量的夹角
		if (i > 0) {
			Eigen::Vector3f prev_normal = cloud_with_normals->points[i - 1].getNormalVector3fMap();
			if (prev_normal.hasNaN()) {
				continue;
			}
			// 计算夹角
			float cos_angle = normal.dot(prev_normal) / (normal.norm() * prev_normal.norm());
			// 限制夹角在[-1, 1]范围内，避免数值误差
			cos_angle = std::clamp(cos_angle, -1.0f, 1.0f);
			float angle = std::acos(cos_angle);  // 计算夹角
			angle_differences.push_back(angle);
		}
	}

	// 法向量方差（标准差）
	variance /= cloud_with_normals->points.size();
	float normal_standard_deviation = std::sqrt(variance);

	// 计算法向量夹角的标准差
	float angle_variance = 0.0f;
	for (float angle : angle_differences) {
		angle_variance += angle * angle;
	}
	angle_variance /= angle_differences.size();
	float angle_standard_deviation = std::sqrt(angle_variance);

	// 输出法向量的标准差和法向量角度的标准差
	std::cout << "Normal Vector Standard Deviation: " << normal_standard_deviation << std::endl;
	std::cout << "Normal Vector Angle Standard Deviation: " << angle_standard_deviation << " radians" << std::endl;
}

int main()
{
	
	//----------------------计算思看扫描点云的法向量一致性---------------------
	std::string filePath = "E:\\Repository\\Github\\LearningC-\\resource\\isolate\\1.txt";
	std::vector<Eigen::Vector3d> vNoramls;
	ReadFile(vNoramls, filePath);
	CalcNormalVariance(vNoramls);
	
	//----------------------读取点云---------------------
	pcl::PointCloud<PointT>::Ptr pCloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT>("2.pcd", *pCloud) == -1)   //statue2.pcd
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}

	//----------------------参数设置---------------------
	// 下采样（可选步骤，减少计算量）
	//pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::VoxelGrid<PointT> voxel_grid;
	//voxel_grid.setInputCloud(pCloud);
	//voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);  // 设置体素的尺寸
	//voxel_grid.filter(*cloud_filtered);
	//pcl::PointCloud<PointT>::Ptr pSmoothedCloud(new pcl::PointCloud<PointT>());
	pcl::PointCloud<pcl::PointNormal>::Ptr pSmoothedCloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr pNormalCloud(new pcl::PointCloud<pcl::PointNormal>());
	//----------------------程序执行--------------------
	auto startOp = std::chrono::high_resolution_clock::now();
	//MovingLeastSquares(pSmoothedCloud, pCloud);
	NeighborBasedNormal(pSmoothedCloud, pNormalCloud, pCloud);
	auto endOp = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp = endOp - startOp;
	std::cout << "点云平滑算法用时: " << elapsedOp.count() << " seconds" << std::endl;

	// 判断法向量的一致性
	ComputeNormalVarianceAndAngle(pNormalCloud);
	ComputeNormalVarianceAndAngle(pSmoothedCloud);
	//float rms_error = 0.0;
	//for (size_t i = 0; i < pCloud->points.size(); ++i) {
	//	Eigen::Vector3f normal1 = pNormalCloud->points[i].getNormalVector3fMap();
	//	Eigen::Vector3f normal2 = pSmoothedCloud->points[i].getNormalVector3fMap();
	//	rms_error += std::pow(normal1.dot(normal2), 2);  // RMS of dot product
	//}
	//rms_error = std::sqrt(rms_error / pCloud->points.size());
	//std::cout << "RMS Error: " << rms_error << std::endl;

	//----------------------显示结果---------------------
#if ENABLE_DISPLAY
	cout << "原始点云点数" << pCloud->points.size() << std::endl;
	cout << "平滑后点云点数" << pSmoothedCloud->points.size() << std::endl;
	//VisualizedCloud(pSmoothedCloud, pCloud);
	pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
	GreedyTriangle(*triangles, pCloud, pNormalCloud);
	GreedyTriangle(*triangles, pCloud, pSmoothedCloud);
#endif
}