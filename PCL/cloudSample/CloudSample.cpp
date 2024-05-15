#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/common/common.h>
#include <pcl/console/time.h> 
#include <pcl/visualization/pcl_visualizer.h>

#define ENABLE_DISPLAY 0		// 定义一个宏，用于控制显示状态
const float KEEP_PERCENTAGE = 15;

// 定义结构体用于保存点云中每个点的索引和曲率
struct PointCurvature
{
	int index;
	float curvature;
};

// 根据曲率对点进行排序
bool SortFunction(const PointCurvature& a, const PointCurvature& b)
{
	return a.curvature < b.curvature;
}

// 计算曲率的函数
void ComputeCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
	const pcl::PointCloud<pcl::Normal>::Ptr& normals,
	std::vector<PointCurvature>& curvatureValues,
	size_t startIdx,
	size_t endIdx)
{
	for (size_t i = startIdx; i < endIdx; ++i)
	{
		curvatureValues[i].index = i;
		curvatureValues[i].curvature = normals->points[i].curvature;
	}
}

// 计算曲率1
void CalCurvature1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// 计算法线
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(10);		// 设置法线计算的搜索步长
	ne.compute(*cloud_normals);

	// 定义存储曲率的结构体
	std::vector<PointCurvature> curvatureValues(cloud->points.size());

	// 计算曲率，利用多线程并行计算
#pragma omp parallel for num_threads(8)
	for (int i = 0; i < cloud->points.size(); ++i)
	{
		curvatureValues[i].index = i;
		curvatureValues[i].curvature = cloud_normals->points[i].curvature;
	}
	//pcl::parallel_for(0, cloud->points.size(), [&](size_t startIdx, size_t endIdx) {
	//	ComputeCurvature(cloud, cloud_normals, curvatureValues, startIdx, endIdx);
	//	});

	// 根据曲率排序
	std::sort(curvatureValues.begin(), curvatureValues.end(), SortFunction);

	// 计算保留点的数量（根据百分比参数）
	size_t numPointsToKeep = static_cast<size_t>((KEEP_PERCENTAGE / 100.0) * curvatureValues.size());

	// 保留前 numPointsToKeep 个点
	pcl::PointCloud<pcl::PointXYZ>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
	sampledCloud->points.resize(numPointsToKeep);

#pragma omp parallel for num_threads(8)
	for (int i = 0; i < numPointsToKeep; ++i)
	{
		sampledCloud->points[i] = cloud->points[curvatureValues[i].index];
	}
	sampledCloud->width = numPointsToKeep;
	sampledCloud->height = 1;
	sampledCloud->is_dense = false;

#if ENABLE_DISPLAY
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Cloud curvature sampling"));

	int v1(0);
	MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
	MView->addText("Raw point clouds", 10, 10, "v1_text", v1);
	int v2(0);
	MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
	MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
	MView->addText("Sampled point clouds", 10, 10, "v2_text", v2);

	MView->addPointCloud<pcl::PointXYZ>(cloud, "Raw cloud", v1);
	MView->addPointCloud<pcl::PointXYZ>(sampledCloud, "Sampled cloud", v2);

	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Raw cloud", v1);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "Sampled cloud", v2);

	MView->addCoordinateSystem(1.0);
	MView->initCameraParameters();

	MView->spin();
#endif
	// 保存采样后的点云
	pcl::io::savePCDFile("output.pcd", *sampledCloud);
}

// 计算曲率2
void CalCurvature2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// 计算法线
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(10);		// 设置法线计算的搜索步长
	ne.compute(*cloud_normals);

	// 计算所有点的曲率
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
	pc.setInputCloud(cloud);
	pc.setInputNormals(cloud_normals);
	pc.setSearchMethod(tree);

	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr pCurvature(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pc.setKSearch(10);
	pc.compute(*pCurvature);

	//计算点云的高斯曲率，获取曲率集
	std::vector<PointCurvature> curvatureValues(cloud->points.size());;

#pragma omp parallel for num_threads(8)
	for (int i = 0; i < pCurvature->size(); i++) {
		//平均曲率
		//float curvature = (*pCurvature)[i].pc1;// +(*pCurvature)[i].pc2) / 2;
		//高斯曲率
		float curvature = (*pCurvature)[i].pc1 * (*pCurvature)[i].pc2;
		//pv.cPoint = tempPoint;
		curvatureValues[i].index = i;
		curvatureValues[i].curvature = curvature;
	}

	// 根据曲率排序
	std::sort(curvatureValues.begin(), curvatureValues.end(), SortFunction);

	// 计算保留点的数量（根据百分比参数）
	size_t numPointsToKeep = static_cast<size_t>((KEEP_PERCENTAGE / 100.0) * curvatureValues.size());

	// 保留前 numPointsToKeep 个点
	pcl::PointCloud<pcl::PointXYZ>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
	sampledCloud->points.resize(numPointsToKeep);

#pragma omp parallel for num_threads(8)
	for (int i = 0; i < numPointsToKeep; ++i)
	{
		sampledCloud->points[i] = cloud->points[curvatureValues[i].index];
	}
	sampledCloud->width = numPointsToKeep;
	sampledCloud->height = 1;
	sampledCloud->is_dense = false;

#if ENABLE_DISPLAY
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Cloud curvature sampling"));

	int v1(0);
	MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
	MView->addText("Raw point clouds", 10, 10, "v1_text", v1);
	int v2(0);
	MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
	MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
	MView->addText("Sampled point clouds", 10, 10, "v2_text", v2);

	MView->addPointCloud<pcl::PointXYZ>(cloud, "Raw cloud", v1);
	MView->addPointCloud<pcl::PointXYZ>(sampledCloud, "Sampled cloud", v2);

	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Raw cloud", v1);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "Sampled cloud", v2);

	MView->addCoordinateSystem(1.0);
	MView->initCameraParameters();

	MView->spin();
#endif
}

int main()
{
	// 加载点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud) == -1)		// sac_plane_test.pcd | Scan_0511_1713.pcd
	{
		PCL_ERROR("点云读取失败 \n");
		return (-1);
	}

	pcl::console::TicToc time;
	time.tic();

	CalCurvature1(cloud);
	std::cout << "直接计算曲率采样用时： " << time.toc() / 1000 << " 秒" << std::endl;

	time.tic();
	CalCurvature2(cloud);
	std::cout << "高斯曲率采样用时： " << time.toc() / 1000 << " 秒" << std::endl;

	return 0;
}