#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <chrono>

#define ENABLE_DISPLAY	  1					// 定义一个宏，用于控制显示状态
#define ENABLE_PLANE_INFO 1

using namespace std;

Eigen::Vector3d normal;
double D;

void SEGPlaneFit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::ModelCoefficients::Ptr coefficients_m(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);							  // 设置最大迭代次数
	seg.setDistanceThreshold(0.1);						  // 设定阈值
	//seg.setNumberOfThreads(10);                         // 设置线程数量
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients_m);

	Eigen::VectorXf coefficients;
	coefficients.resize(4);
	coefficients[0] = coefficients_m->values[0]; coefficients[1] = coefficients_m->values[1];
	coefficients[2] = coefficients_m->values[2]; coefficients[3] = coefficients_m->values[3];

#if ENABLE_PLANE_INFO
	cout << "平面模型系数为：\n"
		<< "A=" << coefficients[0] << "\n"
		<< "B=" << coefficients[1] << "\n"
		<< "C=" << coefficients[2] << "\n"
		<< "D=" << coefficients[3] << "\n" << endl;
#endif
	normal.x() = coefficients[0];
	normal.y() = coefficients[1];
	normal.z() = coefficients[2];
	
	D = coefficients[3];
}

void LeastSquareFit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	Eigen::Vector4d centroid;                    // 质心
	Eigen::Matrix3d covariance_matrix;           // 协方差矩阵

	// 计算归一化协方差矩阵和质心
	pcl::computeMeanAndCovarianceMatrix(*cloud, covariance_matrix, centroid);

	// 计算协方差矩阵的特征值与特征向量
	Eigen::Matrix3d eigenVectors;
	Eigen::Vector3d eigenValues;
	pcl::eigen33(covariance_matrix, eigenVectors, eigenValues);

	// 查找最小特征值的位置
	Eigen::Vector3d::Index minRow, minCol;
	eigenValues.minCoeff(&minRow, &minCol);

	// 获取平面方程：AX+BY+CZ+D = 0的系数
	normal = eigenVectors.col(minCol);
	D = -normal.dot(centroid.head<3>());

#if ENABLE_PLANE_INFO
	cout << "平面模型系数为：\n"
		<< "A=" << normal[0] << "\n"
		<< "B=" << normal[1] << "\n"
		<< "C=" << normal[2] << "\n"
		<< "D=" << D << "\n" << endl;
#endif

}

int
main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("plane.pcd", *cloud) == -1)		// sac_plane_test.pcd | Scan_0511_1713.pcd
	{
		PCL_ERROR("点云读取失败 \n");
		return (-1);
	}

	auto startOp = std::chrono::high_resolution_clock::now();
	SEGPlaneFit(cloud);
	auto endOp = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp3 = endOp - startOp;
	std::cout << "最小二乘平面拟合: " << elapsedOp3.count() << " seconds" << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudModel(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("withModel.pcd", *cloudModel) == -1)		// sac_plane_test.pcd | Scan_0511_1713.pcd
	{
		PCL_ERROR("点云读取失败 \n");
		return (-1);
	}

	// 分割后的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr segCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < cloudModel->size(); ++i)
	{
		Eigen::Vector3d point(cloudModel->points[i].x, cloudModel->points[i].y, cloudModel->points[i].z);
		if ((point.dot(normal) + D) > 0)
		{
			segCloud->points.emplace_back(cloudModel->points[i]);
		}
	}

#if ENABLE_DISPLAY
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud plane fit"));

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v1);
	viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
	viewer->setBackgroundColor(0.5, 0.5, 0.5, v2);
	viewer->addText("Seged cloud", 10, 10, "v2_text", v2);

	viewer->addPointCloud<pcl::PointXYZ>(cloudModel, "Raw cloud", v1);
	viewer->addPointCloud<pcl::PointXYZ>(segCloud, "Plane fit cloud", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Raw cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "Seged cloud", v2);

	//viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
#endif
    return 0;
}

