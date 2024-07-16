#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common.h>
#include <pcl/console/time.h> 

using namespace std;
using namespace pcl::io;
using namespace pcl::console;

#define ENABLE_DISPLAY 1		// 定义一个宏，用于控制显示状态

int
main(int argc, char** argv)
{
	// 加载点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("cone.pcd", *cloud) == -1)		
	{
		PCL_ERROR("点云读取失败 \n");
		return (-1);
	}
	pcl::console::TicToc tt;
	print_highlight("Cone fit: ");
	print_value("%s ", "cone.pcd");
	tt.tic();
	//-------------------------------提取圆锥体模型------------------------------
	// Estimate point normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50); // Adjust depending on your dataset
	normal_estimator.setNumberOfThreads(4);
	normal_estimator.compute(*normals);

	// Create the segmentation object
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

	// Optional - set the model type (CONE)
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CONE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100000);
	seg.setDistanceThreshold(5); // Adjust according to your dataset(影响拟合点数)

	// Segment the largest cone from the cloud
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		return -1;
	}

	// 从RANSAC结果中提取初始参数
	pcl::ExtractIndices<pcl::PointXYZ > extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ >);
	extract.filter(*cloud_cylinder);

	print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms: "); print_value("%d", cloud->width * cloud->height); print_info(" points]\n");

#if ENABLE_DISPLAY
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Cloud Cone"));

	int v1(0);
	MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
	MView->addText("Raw point clouds", 10, 10, "v1_text", v1);

	int v2(0);
	MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
	MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
	MView->addText("Fitted point clouds", 10, 10, "v2_text", v2);

	MView->addPointCloud<pcl::PointXYZ>(cloud, "Raw cloud", v1);
	MView->addPointCloud<pcl::PointXYZ>(cloud_cylinder, "Cone cloud", v2);

	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Raw cloud", v1);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "Cone cloud", v2);

	MView->initCameraParameters();
	MView->spin();
#endif
	return 0;
}

