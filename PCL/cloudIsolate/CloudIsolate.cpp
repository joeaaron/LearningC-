#include <iostream>
#include <pcl/io/pcd_io.h>  
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>  

using namespace std;

#define ENABLE_DISPLAY 1				// 定义一个宏，用于控制显示状态

// 点云曲率
struct PointCurvature
{
	int index;							// 索引
	double curvature;					// 曲率
};

void PCLSorFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(2.6);				// 考虑周围多少个点，找到足够的邻域点
	sor.setStddevMulThresh(1);
	sor.setNegative(true);
	sor.filter(*filteredCloud);
}

int main()
{
	//----------------------读取点云---------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("2.pcd", *pCloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	//----------------------参数设置---------------------
	vector<PointCurvature> vPointCurvature;
	double dRatio = 0.1;

	//----------------------程序执行---------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr pFiltered(new pcl::PointCloud<pcl::PointXYZ>);
	PCLSorFilter(pFiltered, pCloud);

	//----------------------显示结果---------------------
#if ENABLE_DISPLAY
//---------------------------结果可视化----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Cloud Filter"));

	// 设置单个视口背景颜色
	MView->setBackgroundColor(0.3, 0.3, 0.3);

	// 添加原始点云并设置为蓝灰色
	MView->addPointCloud<pcl::PointXYZ>(pCloud, "Raw cloud");
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.2627, 0.3451, 0.4588, "Raw cloud");

	// 添加过滤后的点云并设置为红色
	MView->addPointCloud<pcl::PointXYZ>(pFiltered, "Filtered cloud");
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Filtered cloud");

	// 添加坐标系（可选）
	MView->addCoordinateSystem(1.0);

	// 初始化相机参数并重置相机视角
	MView->resetCamera();
	MView->initCameraParameters();

	// 开始交互式可视化
	MView->spin();
#endif
}