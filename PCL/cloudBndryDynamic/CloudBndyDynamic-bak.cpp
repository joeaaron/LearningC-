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

// 定义一个结构来存储点云数据和边界信息
struct PointCloudData {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	pcl::PointCloud<pcl::Boundary>::Ptr boundaries;
	std::unordered_set<int> boundary_indices;				// 存储边界点的索引
};

// 生成示例点云数据
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

// 计算法向量和曲率
void computeNormalsAndCurvature(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::Normal>::Ptr normals) {
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	ne.setKSearch(10);
	ne.compute(*normals);
}

// 计算边界
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

// 更新边界点状态
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

		// 检查邻域内是否有新的点
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
			// 重新计算这个点的法向量和曲率
			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
			ne.setInputCloud(data.cloud);
			ne.setSearchMethod(tree);
			pcl::PointCloud<pcl::Normal>::Ptr point_normal(new pcl::PointCloud<pcl::Normal>());
			ne.setKSearch(10);
			ne.compute(*point_normal);

			// 判断是否仍然为边界点
			pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> be;
			be.setInputCloud(data.cloud);
			be.setInputNormals(point_normal);
			pcl::PointCloud<pcl::Boundary>::Ptr point_boundary(new pcl::PointCloud<pcl::Boundary>());
			be.setKSearch(10);
			be.compute(*point_boundary);

			if (!point_boundary->points[0].boundary_point) 
			{
				//如果不再是边界点，将其从集合中移除
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

// 更新点云数据，动态检测和更新边界
void updatePointCloud(PointCloudData& data, pcl::PointCloud<pcl::PointXYZ>::Ptr new_data) 
{
	// 记录新增点的索引
	int start_idx = data.cloud->points.size();
	*data.cloud += *new_data;
	int end_idx = data.cloud->points.size();
	std::vector<int> new_points_indices;
	for (int i = start_idx; i < end_idx; ++i)
	{
		new_points_indices.push_back(i);
	}

	// 计算新增点的法向量和曲率
	pcl::PointCloud<pcl::Normal>::Ptr new_normals(new pcl::PointCloud<pcl::Normal>());
	computeNormalsAndCurvature(new_data, new_normals);

	// 将新增点的法向量合并到总体法向量中
	*data.normals += *new_normals;

	// 计算新增点的边界
	pcl::PointCloud<pcl::Boundary>::Ptr new_boundaries(new pcl::PointCloud<pcl::Boundary>());
	computeBoundaries(new_data, new_normals, new_boundaries);

	// 更新边界点集合
	for (size_t i = 0; i < new_boundaries->points.size(); ++i) 
	{
		if (new_boundaries->points[i].boundary_point) 
		{
			data.boundary_indices.insert(start_idx + i);
		}
	}

	// 更新原有边界点的状态
	updateBoundaryStatus(data, new_points_indices);
}

int main(int argc, char** argv) 
{
	// 初始化点云数据
	PointCloudData data;
	data.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
	data.normals.reset(new pcl::PointCloud<pcl::Normal>());
	data.boundaries.reset(new pcl::PointCloud<pcl::Boundary>());

	// 生成初始点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr initial_data = generatePointCloud(100);
	*data.cloud = *initial_data;

	// 计算初始法向量和边界
	computeNormalsAndCurvature(data.cloud, data.normals);
	computeBoundaries(data.cloud, data.normals, data.boundaries);

	// 更新初始边界点集合
	for (size_t i = 0; i < data.boundaries->points.size(); ++i) 
	{
		if (data.boundaries->points[i].boundary_point) 
		{
			data.boundary_indices.insert(i);
		}
	}

	// 模拟增量更新
	for (int i = 1; i <= 50; ++i)
	{
		// 生成新的增量点云数据
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_data = generatePointCloud(20, 0.5 * i);

		// 更新点云并动态检测和更新边界
		updatePointCloud(data, new_data);

		// 输出当前边界点数量
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

		//-----------------结果显示---------------------
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