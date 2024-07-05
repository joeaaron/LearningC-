#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/console/print.h>
#include <pcl/common/common.h>
#include <pcl/console/time.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#define ENABLE_DISPLAY 1					// 定义一个宏，用于控制显示状态
#define ENABLE_TEST 0						// 定义一个宏，用于控制算法测试

const float KEEP_PERCENTAGE = 15;
const int TESTS_NUM = 10;

using namespace pcl::io;
using namespace pcl::console;

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

// 曲率采样后显示
void CurvatureSample(std::vector<PointCurvature>& curvatureValues, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
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

	//MView->addCoordinateSystem(1.0);
	MView->initCameraParameters();

	MView->spin();
#endif
	// 保存采样后的点云
	pcl::io::savePCDFile("output.pcd", *sampledCloud);
}

// 计算曲率1
void CalCurvature1(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	// 计算法线
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(10);				// 设置法线计算的搜索步长
	ne.setNumberOfThreads(4);
	ne.compute(*cloudNormals);

	// 定义存储曲率的结构体
	std::vector<PointCurvature> curvatureValues(cloud->points.size());

	// 计算曲率，利用多线程并行计算
#pragma omp parallel for num_threads(4)
	for (int i = 0; i < cloud->points.size(); ++i)
	{
		curvatureValues[i].index = i;
		curvatureValues[i].curvature = cloudNormals->points[i].curvature;
	}

	CurvatureSample(curvatureValues, cloud);
}

// 计算曲率2
void CalCurvature2(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	// 计算法线
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(10);		// 设置法线计算的搜索步长
	unsigned int num_threads = std::thread::hardware_concurrency();
	ne.setNumberOfThreads(num_threads);
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

	CurvatureSample(curvatureValues, cloud);
}

// 计算法线和曲率
void computeNormalsAndCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
	pcl::PointCloud<pcl::Normal>::Ptr& normals,
	std::vector<PointCurvature>& curvatureValues,
	std::mutex& mtx,
	size_t startIdx,
	size_t endIdx)
{
	// 创建法线估计器
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	// 设置法线计算的搜索方法和搜索半径
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	ne.setKSearch(10);		// 设置法线计算的搜索步长

	// 计算法线
	normals.reset(new pcl::PointCloud<pcl::Normal>);
	ne.compute(*normals);

	// 计算曲率并存储结果
	for (size_t i = startIdx; i < endIdx; ++i)
	{
		std::lock_guard<std::mutex> lk(mtx);

		curvatureValues[i].index = i;
		curvatureValues[i].curvature = normals->points[i].curvature;
	}
}

// 计算曲率3
void CalCurvature3(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{

	// 定义存储法线和曲率的容器
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	std::vector<PointCurvature> curvatureValues(cloud->size());

	// 使用互斥锁保护共享数据 curvatureValues
	std::mutex mtx;

	// 获取可用的 CPU 核心数
	unsigned int num_threads = std::thread::hardware_concurrency();

//	// 并行计算法线和曲率
//#pragma omp parallel sections
//	{
//#pragma omp section
//		{
//			// 计算法线
//			pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
//			ne.setInputCloud(cloud);
//			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//			ne.setSearchMethod(tree);
//			ne.setKSearch(10);
//			ne.compute(*normals);
//
//			// 定义存储曲率的结构体
//			std::vector<PointCurvature> curvatureValues(cloud->points.size());
//			for (int i = 0; i < cloud->points.size(); ++i)
//			{
//				curvatureValues[i].index = i;
//				curvatureValues[i].curvature = normals->points[i].curvature;
//			}
//		}
//	}
	std::vector<std::thread> threads(num_threads);
	size_t chunk_size = cloud->size() / num_threads;
	size_t startIdx = 0;
	for (unsigned int i = 0; i < num_threads; ++i)
	{
		size_t endIdx = (i == num_threads - 1) ? cloud->size() : startIdx + chunk_size;
		threads[i] = std::thread([&cloud, &normals, &curvatureValues, &mtx, startIdx, endIdx]() {
			computeNormalsAndCurvature(cloud, normals, curvatureValues, mtx, startIdx, endIdx);
			});
		startIdx = endIdx;
	}

	// 等待所有线程结束
	for (auto& t : threads)
	{
		t.join();
	}

	CurvatureSample(curvatureValues, cloud);
}

void PCLVoxelSample(pcl::PointCloud<pcl::PointXYZ>::Ptr& sampledCloud, 
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(1, 1, 1);
	vg.filter(*sampledCloud);
}

/**
 * @description:			体素滤波
 * @param cloud				输入点云
 * @param cloud_filtered	滤波点云
 * @param leafsize			体素大小
 */
void VoxelSample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudSampled, 
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudIn, float leafsize)
{
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*cloudIn, minPt, maxPt);

	float lx = maxPt.x - minPt.x;
	float ly = maxPt.y - minPt.y;
	float lz = maxPt.z - minPt.z;

	int nx = static_cast<int>(std::ceil(lx / leafsize));
	int ny = static_cast<int>(std::ceil(ly / leafsize));
	int nz = static_cast<int>(std::ceil(lz / leafsize));

	std::vector<pcl::PointCloud<pcl::PointXYZ>> v(nx * ny * nz);

	for (int i = 0; i < cloudIn->points.size(); i++)
	{
		int ix = static_cast<int>(std::floor((cloudIn->points[i].x - minPt.x) / leafsize));
		int iy = static_cast<int>(std::floor((cloudIn->points[i].y - minPt.y) / leafsize));
		int iz = static_cast<int>(std::floor((cloudIn->points[i].z - minPt.z) / leafsize));

		// 注意索引计算的修正
		v[ix + iy * nx + iz * (nx * ny)].push_back(cloudIn->points[i]);
	}

	for (int i = 0; i < v.size(); ++i)
	{
		if (!v[i].empty())
		{
			Eigen::Vector4f centroid;
			pcl::compute3DCentroid(v[i], centroid);
			cloudSampled->push_back(pcl::PointXYZ(centroid.x(), centroid.y(), centroid.z()));
		}
	}
}

int main()
{
	// 加载点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("1600w.pcd", *cloud) == -1)		//
	{
		PCL_ERROR("点云读取失败 \n");
		return (-1);
	}

	pcl::console::TicToc tt;
	//time.tic();
	//CalCurvature1(cloud);
	//std::cout << "直接曲率采样用时： " << time.toc() / 1000 << " 秒" << std::endl;
	//
	//time.tic();
	//CalCurvature2(cloud);
	//std::cout << "高斯曲率采样用时： " << time.toc() / 1000 << " 秒" << std::endl;
	//
	//time.tic();
	//CalCurvature3(cloud);
	//std::cout << "多线程直接曲率采样用时： " << time.toc() / 1000 << " 秒" << std::endl;
	//
	
	print_highlight("Sampling: "); 
	print_value("%s ", "table_scene_lms400.pcd");
	tt.tic();
	pcl::PointCloud<pcl::PointXYZ>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
	VoxelSample(sampledCloud, cloud, 1);
	//PCLVoxelSample(sampledCloud, cloud);
	//std::cout << "体素采样用时： " << time.toc() / 1000 << " 秒" << std::endl;
	print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms: "); print_value("%d", sampledCloud->width * sampledCloud->height); print_info(" points]\n");

#if ENABLE_TEST
	// 算法测试框架
	std::vector<double> vTimes;
	double dMaxTime = 0.0;

	for (int i = 0; i < TESTS_NUM; ++i)
	{
		auto start = std::chrono::high_resolution_clock::now();
		CalCurvature1(cloud);
		auto end = std::chrono::high_resolution_clock::now();

		std::chrono::duration<double, std::milli> duration = end - start;
		vTimes.emplace_back(duration.count());

		if (duration.count() > dMaxTime)
		{
			dMaxTime = duration.count();
		}
	}
	
	double dAverageTime = std::accumulate(vTimes.begin(), vTimes.end(), 0.0) / vTimes.size();
	std::cout << "直接曲率采样平均用时： " << dAverageTime << "ms" << std::endl;
	std::cout << "直接曲率采样最大用时： " << dMaxTime << " ms" << std::endl;

	//std::cout << "高斯曲率采样平均用时： " << dAverageTime << "ms" << std::endl;
	//std::cout << "高斯曲率采样最大用时： " << dMaxTime << " ms" << std::endl;
#endif	

#if ENABLE_DISPLAY
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Cloud Sample"));

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

	MView->initCameraParameters();
	MView->spin();
#endif

	return 0;
}