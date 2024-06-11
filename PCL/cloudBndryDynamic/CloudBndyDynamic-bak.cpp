#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <unordered_set>
#include <vector>
#include <iostream>
#include <random>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

// ����һ���ṹ���洢�������ݺͱ߽���Ϣ
struct PointCloudData {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	pcl::PointCloud<pcl::Boundary>::Ptr boundaries;
	std::unordered_set<int> boundary_indices;				// �洢�߽�������
};

// ����ʾ����������
pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointCloud(int num_points, float x_offset = 0.0) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0.0, 1.0);

	for (int i = 0; i < num_points; ++i) {
		pcl::PointXYZ point;
		point.x = distribution(generator) + x_offset;
		point.y = distribution(generator);
		point.z = distribution(generator);
		cloud->points.push_back(point);
	}

	cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->is_dense = true;

	return cloud;
}

// ���㷨����������
void computeNormalsAndCurvature(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::Normal>::Ptr normals) {
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	ne.setKSearch(10);
	ne.compute(*normals);
}

// ����߽�
void computeBoundaries(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::Normal>::Ptr normals,
	pcl::PointCloud<pcl::Boundary>::Ptr boundaries) {
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> be;
	be.setInputCloud(cloud);
	be.setInputNormals(normals);
	be.setKSearch(10);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	be.setSearchMethod(tree);
	be.compute(*boundaries);
}

// ���±߽��״̬
void updateBoundaryStatus(PointCloudData& data, std::vector<int>& new_points_indices) 
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(data.cloud);

	for (auto it = data.boundary_indices.begin(); it != data.boundary_indices.end();) 
	{
		int idx = *it;
		std::vector<int> point_indices;
		std::vector<float> point_distances;
		tree->nearestKSearch(data.cloud->points[idx], 10, point_indices, point_distances);

		// ����������Ƿ����µĵ�
		bool has_new_neighbors = false;
		for (int point_idx : point_indices) 
		{
			if (std::find(new_points_indices.begin(), new_points_indices.end(), point_idx) != new_points_indices.end()) {
				has_new_neighbors = true;
				break;
			}
		}

		if (has_new_neighbors) 
		{
			// ���¼��������ķ�����������
			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
			ne.setInputCloud(data.cloud);
			ne.setSearchMethod(tree);
			pcl::PointCloud<pcl::Normal>::Ptr point_normal(new pcl::PointCloud<pcl::Normal>());
			ne.setKSearch(10);
			ne.compute(*point_normal);

			// �ж��Ƿ���ȻΪ�߽��
			pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> be;
			be.setInputCloud(data.cloud);
			be.setInputNormals(point_normal);
			pcl::PointCloud<pcl::Boundary>::Ptr point_boundary(new pcl::PointCloud<pcl::Boundary>());
			be.setKSearch(10);
			be.compute(*point_boundary);

			if (!point_boundary->points[0].boundary_point) 
			{
				//��������Ǳ߽�㣬����Ӽ������Ƴ�
				it = data.boundary_indices.erase(it);
			}
			else 
			{
				++it;
			}
		}
		else 
		{
			++it;
		}
	}
}

// ���µ������ݣ���̬���͸��±߽�
void updatePointCloud(PointCloudData& data, pcl::PointCloud<pcl::PointXYZ>::Ptr new_data) 
{
	// ��¼�����������
	int start_idx = data.cloud->points.size();
	*data.cloud += *new_data;
	int end_idx = data.cloud->points.size();
	std::vector<int> new_points_indices;
	for (int i = start_idx; i < end_idx; ++i)
	{
		new_points_indices.push_back(i);
	}

	// ����������ķ�����������
	pcl::PointCloud<pcl::Normal>::Ptr new_normals(new pcl::PointCloud<pcl::Normal>());
	computeNormalsAndCurvature(new_data, new_normals);

	// ��������ķ������ϲ������巨������
	*data.normals += *new_normals;

	// ����������ı߽�
	pcl::PointCloud<pcl::Boundary>::Ptr new_boundaries(new pcl::PointCloud<pcl::Boundary>());
	computeBoundaries(new_data, new_normals, new_boundaries);

	// ���±߽�㼯��
	for (size_t i = 0; i < new_boundaries->points.size(); ++i) 
	{
		if (new_boundaries->points[i].boundary_point) 
		{
			data.boundary_indices.insert(start_idx + i);
		}
	}

	// ����ԭ�б߽���״̬
	updateBoundaryStatus(data, new_points_indices);
}

int main(int argc, char** argv) 
{
	// ��ʼ����������
	PointCloudData data;
	data.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
	data.normals.reset(new pcl::PointCloud<pcl::Normal>());
	data.boundaries.reset(new pcl::PointCloud<pcl::Boundary>());

	// ���ɳ�ʼ��������
	pcl::PointCloud<pcl::PointXYZ>::Ptr initial_data = generatePointCloud(100);
	*data.cloud = *initial_data;

	// �����ʼ�������ͱ߽�
	computeNormalsAndCurvature(data.cloud, data.normals);
	computeBoundaries(data.cloud, data.normals, data.boundaries);

	// ���³�ʼ�߽�㼯��
	for (size_t i = 0; i < data.boundaries->points.size(); ++i) 
	{
		if (data.boundaries->points[i].boundary_point) 
		{
			data.boundary_indices.insert(i);
		}
	}

	// ģ����������
	for (int i = 1; i <= 50; ++i)
	{
		// �����µ�������������
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_data = generatePointCloud(20, 0.5 * i);

		// ���µ��Ʋ���̬���͸��±߽�
		updatePointCloud(data, new_data);

		// �����ǰ�߽������
		std::cout << "After update " << i << ": " << data.boundary_indices.size() << " boundary points." << std::endl;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud_visual->resize(data.cloud->size());
		for (size_t i = 0; i < data.cloud->size(); i++)
		{
			cloud_visual->points[i].x = data.cloud->points[i].x;
			cloud_visual->points[i].y = data.cloud->points[i].y;
			cloud_visual->points[i].z = data.cloud->points[i].z;
			if (data.boundary_indices.find(i) != data.boundary_indices.end())
			{
				cloud_visual->points[i].r = 255;
				cloud_visual->points[i].g = 0;
				cloud_visual->points[i].b = 0;
				cloud_boundary->push_back(cloud_visual->points[i]);
			}
			else
			{
				cloud_visual->points[i].r = 255;
				cloud_visual->points[i].g = 255;
				cloud_visual->points[i].b = 255;
			}
		}

		//-----------------�����ʾ---------------------
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud boundary extraction - AS"));

		int v1(0), v2(0);
		viewer->setWindowName("Cloud boundary extraction - ACEX");
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud(data.cloud, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

		viewer->addPointCloud(cloud_boundary, "cloud_boundary");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_boundary");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_boundary");

		while (!viewer->wasStopped())
		{
			viewer->spinOnce(1000);
		}
	}

	return 0;
}