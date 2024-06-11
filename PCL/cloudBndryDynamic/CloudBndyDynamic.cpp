#include <iostream>
#include <vector>
#include <random>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/boundary.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;

void addSimulatedPoints(PointCloud<PointXYZ>::Ptr& cloud, size_t x_offset) 
{
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0.0, 1.0);

	for (int i = 0; i < 1000; ++i) {
		pcl::PointXYZ point;
		point.x = distribution(generator) + x_offset;
		point.y = distribution(generator);
		point.z = distribution(generator);
		cloud->points.push_back(point);
	}

	cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->is_dense = true;
}

void boundaryDetection(PointCloud<PointXYZ>::Ptr cloud, std::vector<bool>& boundary_flags, size_t start, size_t end) 
{
	// ����һ���µ��ӵ���
	PointCloud<PointXYZ>::Ptr sub_cloud(new PointCloud<PointXYZ>());
	for (size_t j = start; j < end; ++j) 
	{
		sub_cloud->points.push_back(cloud->points[j]);
	}
	sub_cloud->width = sub_cloud->points.size();
	sub_cloud->height = 1;
	sub_cloud->is_dense = true;

	// ���㷨��
	NormalEstimationOMP<PointXYZ, Normal> ne;
	ne.setInputCloud(sub_cloud);
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
	ne.setSearchMethod(tree);
	PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());
	ne.setKSearch(35);
	ne.setNumberOfThreads(4);
	ne.compute(*cloud_normals);

	// ���߽�
	BoundaryEstimation<PointXYZ, Normal, Boundary> be;
	be.setInputCloud(sub_cloud);
	be.setInputNormals(cloud_normals);
	be.setKSearch(35);
	PointCloud<Boundary>::Ptr boundaries(new PointCloud<Boundary>());
	be.compute(*boundaries);

	// �趨�߽��־λ
	for (size_t j = 0; j < sub_cloud->points.size(); ++j) 
	{
		if (boundaries->points[j].boundary_point > 0) 
		{
			boundary_flags[start + j] = true;
		}
		else 
		{
			boundary_flags[start + j] = false;
		}
	}
}

int main(int argc, char** argv) 
{
	// �������ƶ���
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr boundary_points(new PointCloud<PointXYZ>());

	// ��ʼ���߽��־λ����
	std::vector<bool> boundary_flags;

	// ���ò���
	int interval = 50;
	int stepSize = 5;
	int previousPointCount = 0;
	for (int K = 1; K <= 5; ++K) 
	{
		// ģ��ʵʱɨ����̣����ӵ�������
		addSimulatedPoints(cloud, 0.5*K);

		// ���±߽��־λ�����С
		boundary_flags.resize(cloud->points.size(), false);

		// �������߽磨�������ڣ�
		int newPointsCount = cloud->points.size();
		if (newPointsCount > interval)
		{
			for (size_t i = previousPointCount; i <= newPointsCount - interval; i += stepSize)
			{
				boundaryDetection(cloud, boundary_flags, i, i + interval);
			}
		}
		previousPointCount = newPointsCount - interval;

		// ���±߽����
		boundary_points->points.clear();
		for (size_t i = 0; i < cloud->points.size(); ++i) 
		{
			if (boundary_flags[i]) 
			{
				boundary_points->points.push_back(cloud->points[i]);
			}
		}

		//-----------------�����ʾ---------------------
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud boundary extraction - AS"));

		int v1(0), v2(0);
		viewer->setWindowName("Cloud boundary extraction - AC");
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud(cloud, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

		viewer->addPointCloud(boundary_points, "cloud_boundary");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_boundary");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_boundary");

		while (!viewer->wasStopped())
		{
			viewer->spinOnce(1000);
		}
	}

	return 0;
}