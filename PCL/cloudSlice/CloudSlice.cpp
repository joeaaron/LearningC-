#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <omp.h> // 引入OpenMP支持

int main(int argc, char** argv)
{
	// 创建并填充点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// 假设 cloud 已经被填充了三角形网格的顶点数据

	// 定义截取平面的参数
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4); // 平面方程 ax + by + cz + d = 0
	coefficients->values[0] = 1;/* 平面法向量x分量 */;
	coefficients->values[1] = 0;/* 平面法向量y分量 */;
	coefficients->values[2] = 0;/* 平面法向量z分量 */;
	coefficients->values[3] = 1;/* 平面到原点的距离 */;

	// 创建分割对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return -1;
	}

	// 使用OpenMP并行提取分割后的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
#pragma omp parallel for
	for (int i = 0; i < inliers->indices.size(); ++i)
	{
		int index = inliers->indices[i];
		pcl::PointXYZ point = cloud->points[index];
#pragma omp critical
		cloud_filtered->push_back(point);
	}

	// cloud_filtered 现在包含了截取平面上的点云数据
	// 根据需要进一步处理 cloud_filtered

	return 0;
}