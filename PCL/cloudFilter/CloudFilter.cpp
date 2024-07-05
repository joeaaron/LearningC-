#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>  
#include <pcl/common/common.h>
#include <pcl/console/time.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#define ENABLE_DISPLAY 1					// 定义一个宏，用于控制显示状态
#define ENABLE_TEST 1						// 定义一个宏，用于控制算法测试

const int TESTS_NUM = 20;

using namespace pcl::io;
using namespace pcl::console;

void PCLSorFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud, 
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(20);
	sor.setStddevMulThresh(1);
	sor.filter(*filteredCloud);
}

/**
 * @description:	计算向量元素的均值
 * @param v			输入向量
 * @return			向量元素的均值
 */
float DistAvg(std::vector<float>& v)
{
	float sum = 0.0;
	for (int i = 0; i < v.size(); ++i)
	{
		sum += sqrt(v[i]);
	}
	return sum / v.size();
}

/**
 * @description:	计算向量元素的标准差
 * @param v			输入向量
 * @param avg		向量元素的均值
 * @return			向量元素的标准差
 */
float CalcSigma(std::vector<float>& v, float& avg)
{
	float sigma = 0.0;
	for (int i = 0; i < v.size(); ++i)
	{
		sigma += pow(v[i] - avg, 2);
	}
	return sqrt(sigma / v.size());
}

/**
 * @description:			统计学滤波
 * @param cloud				输入点云
 * @param cloud_filtered	滤波点云
 * @param nr_k				k邻近点数
 * @param std_mul			标准差乘数
 * @param negative			设置移除或者保留
 */
//void StatisticalRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered,
//	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
//	int nr_k, double std_mul, bool negative = false)
//{
//	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
//	tree.setInputCloud(cloud);
//	std::vector<float> avg;
//#pragma omp parallel for
//	for (int i = 0; i < cloud->points.size(); ++i)
//	{
//		std::vector<int> id(nr_k);
//		std::vector<float> dist(nr_k);
//		tree.nearestKSearch(cloud->points[i], nr_k, id, dist);
//#pragma omp critical
//		{
//			avg.push_back(DistAvg(dist));
//		}
//	}
//
//	float u = accumulate(avg.begin(), avg.end(), 0.0) / avg.size();
//	float sigma = CalcSigma(avg, u);
//	std::vector<int> index;
//#pragma omp parallel for
//	for (int i = 0; i < cloud->points.size(); ++i)
//	{
//		std::vector<int> id(nr_k);
//		std::vector<float> dist(nr_k);
//		tree.nearestKSearch(cloud->points[i], nr_k, id, dist);
//		float temp_avg = DistAvg(dist);
//		if (temp_avg >= u - std_mul * sigma && temp_avg <= u + std_mul * sigma)
//#pragma omp critical
//		{
//			index.push_back(i);
//		}
//	}
//
//	//boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index);
//	pcl::IndicesPtr index_ptr(new std::vector<int>(index));
//	pcl::ExtractIndices<pcl::PointXYZ> extract;
//	extract.setInputCloud(cloud);
//	extract.setIndices(index_ptr);
//	extract.setNegative(negative);
//	extract.filter(*cloud_filtered);
//}

template <typename PointT>
void StatisticalRemoval(std::shared_ptr<pcl::PointCloud<PointT>>& pCloudOut,
	const std::shared_ptr<pcl::PointCloud<PointT>>& pCloudIn,
	int nr_k, double std_mul,
	bool negative)
{
	pcl::KdTreeFLANN<PointT> tree;
	tree.setInputCloud(pCloudIn);

	std::vector<float> avg;
#pragma omp parallel for
	for (int i = 0; i < pCloudIn->points.size(); ++i)
	{
		std::vector<int> id(nr_k);
		std::vector<float> dist(nr_k);
		tree.nearestKSearch(pCloudIn->points[i], nr_k, id, dist);
#pragma omp critical
		{
			avg.push_back(DistAvg(dist));
		}
	}

	float u = accumulate(avg.begin(), avg.end(), 0.0) / avg.size();
	float sigma = CalcSigma(avg, u);
	std::vector<int> index;
#pragma omp parallel for
	for (int i = 0; i < pCloudIn->points.size(); ++i)
	{
		std::vector<int> id(nr_k);
		std::vector<float> dist(nr_k);
		tree.nearestKSearch(pCloudIn->points[i], nr_k, id, dist);
		float avgTmp = DistAvg(dist);
		if (avgTmp >= u - std_mul * sigma && avgTmp <= u + std_mul * sigma)
#pragma omp critical
		{
			index.push_back(i);
		}
	}

	pcl::IndicesPtr idxPtr(new std::vector<int>(index));
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(pCloudIn);
	extract.setIndices(idxPtr);
	extract.setNegative(negative);
	extract.filter(*pCloudOut);
}

int main()
{
	// 加载点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud) == -1)		// table_scene_lms400.pcd | rabbit.pcd | 800w
	{
		PCL_ERROR("点云读取失败 \n");
		return (-1);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

	//auto start = std::chrono::high_resolution_clock::now();
	//auto end = std::chrono::high_resolution_clock::now();
	//std::chrono::duration<double, std::milli> duration = end - start;
	///*std::cout << "统计滤波用时： " << duration.count() << "ms" << std::endl;*/

	//TicToc tt;
	//print_highlight("Filtering "); print_value("%s ", "table_scene_lms400.pcd");
	//tt.tic();
	//
	////PCLSorFilter(filteredCloud, cloud);
	//StatisticalRemoval<pcl::PointXYZ>(filteredCloud, cloud, 20, 1, false);
	//print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms: "); print_value("%d", cloud->width * cloud->height); print_info(" points]\n");

#if ENABLE_TEST
	// 算法测试框架
	std::vector<double> vTimes;
	double dMaxTime = 0.0;

	for (int i = 0; i < TESTS_NUM; ++i)
	{
		auto start = std::chrono::high_resolution_clock::now();
		PCLSorFilter(filteredCloud, cloud);
		//StatisticalRemoval<pcl::PointXYZ>(filteredCloud, cloud, 20, 1, false);
		auto end = std::chrono::high_resolution_clock::now();

		std::chrono::duration<double, std::milli> duration = end - start;
		vTimes.emplace_back(duration.count());

		if (duration.count() > dMaxTime)
		{
			dMaxTime = duration.count();
		}
	}

	double dAverageTime = std::accumulate(vTimes.begin(), vTimes.end(), 0.0) / vTimes.size();
	std::cout << "平均用时： " << dAverageTime << "ms" << std::endl;
	std::cout << "最大用时： " << dMaxTime << " ms" << std::endl;
#endif

#if ENABLE_DISPLAY
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Cloud Filter"));

	int v1(0);
	MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
	MView->addText("Raw point clouds", 10, 10, "v1_text", v1);
	int v2(0);
	MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
	MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
	MView->addText("Filtered point clouds", 10, 10, "v2_text", v2);

	MView->addPointCloud<pcl::PointXYZ>(cloud, "Raw cloud", v1);
	MView->addPointCloud<pcl::PointXYZ>(filteredCloud, "Filtered cloud", v2);

	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Raw cloud", v1);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "Filtered cloud", v2);

	//MView->addCoordinateSystem(1.0);
	MView->initCameraParameters();

	MView->spin();
#endif

	return 0;
}