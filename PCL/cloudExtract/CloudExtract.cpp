#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <chrono>

typedef pcl::PointXYZ PointT;

// 计算点到圆锥轴的距离
double distanceToConeAxis(const PointT& point, const Eigen::Vector3f& cone_vertex, const Eigen::Vector3f& cone_axis) 
{
	// 归一化 cone_axis 确保它是单位向量
	Eigen::Vector3f normalized_axis = cone_axis.normalized();

	Eigen::Vector3f point_vec = point.getVector3fMap() - cone_vertex;
	Eigen::Vector3f projection = point_vec.dot(normalized_axis) * normalized_axis;
	double distance = (point_vec - projection).norm();
	return distance;
}

// 判断点是否在圆锥的高度范围内(和轴向夹角为锐角且在轴向上的投影长度<=height)
bool WithinConeHeight(const PointT& point, 
	const Eigen::Vector3f& cone_vertex, 
	const Eigen::Vector3f& cone_axis, 
	double apexDistance, double height)
{
	//Eigen::Vector3f point_vec = point.getVector3fMap() - cone_vertex;
	//double distance_along_axis = point_vec.dot(cone_axis) + apexDistance;				// 点在圆锥轴方向上的投影长度
	auto distance = point.z - apexDistance;  // 浮点型相减有精度损失
	return distance >= 0 && ((distance - height) <= 0.01);
}

// 计算点到圆锥体的最短距离
double ShortestDistanceToCone(const PointT& point, 
	const Eigen::Vector3f& cone_vertex,
	const Eigen::Vector3f& cone_axis,
	double slope) 
{
	Eigen::Vector3f point_vec = point.getVector3fMap() - cone_vertex;
	double distance_along_axis = point_vec.dot(cone_axis);  

	double radius_at_point = distance_along_axis * std::tan(slope);					// 当前高度的圆锥半径
	double distance_to_axis = distanceToConeAxis(point, cone_vertex, cone_axis);

	return std::abs(distance_to_axis - radius_at_point);
}

double findClosestZValueKDTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double target_z) {
	// 创建 KD-Tree 对象并将点云输入
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	// 定义搜索点
	pcl::PointXYZ searchPoint;
	searchPoint.z = target_z;
	searchPoint.x = 0.0f;  // X 和 Y 的值在这里无关紧要
	searchPoint.y = 0.0f;

	// 搜索最接近的一个点
	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
		return cloud->points[pointIdxNKNSearch[0]].z;
	}
	else {
		throw std::runtime_error("No nearest point found.");
	}
}

int main(int argc, char** argv) 
{
	// 1. 读取点云数据
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT>("cube3.pcd", *cloud) == -1) {
		PCL_ERROR("Couldn't read file input.pcd \n");
		return -1;
	}
	std::cout << "Loaded " << cloud->width * cloud->height << " data points"  << std::endl;

	auto startOp = std::chrono::high_resolution_clock::now();

	// 2. 设置圆锥参数
	Eigen::Vector3f cone_vertex(0.000, 101.99999999, -55.98076197);   // 圆锥顶点位置
	Eigen::Vector3f cone_axis(0.000, 0.000, -1);			// 圆锥轴向方向
	cone_axis.normalize();								    // 规范化轴向方向

	double slope = M_PI / 6.0 * 0.5;						// 圆锥的坡度（弧度），即30度
	double apexDistance = 30.98076196;							// 从顶点的距离
	double height = 25.00000002;								    // 圆锥的高度
	
	// 3. 筛选点云中的候选点
	pcl::PointCloud<PointT>::Ptr cloud_cone(new pcl::PointCloud<PointT>);
	std::vector<double> distances;

	// 找到离从顶点最近的Z值
	double closest_z = findClosestZValueKDTree(cloud, cone_vertex[2] + apexDistance);

	for (const auto& point : cloud->points)
	{
		if (WithinConeHeight(point, cone_vertex, cone_axis, closest_z, height))
		{
			// 顶部点要特殊筛选
			if (abs(point.z - closest_z) <= 0.001)
			{
				double radius = (point.z - cone_vertex[2]) * tan(slope);
				double distance = std::sqrt(point.x * point.x + (point.y - cone_vertex[1]) * (point.y - cone_vertex[1]));

				if (((distance - radius) <= 4.078) && ((distance - radius) >= 3.102))  
				{
					cloud_cone->points.push_back(point);
				}
			}
			// 底部点要特殊筛选
			else if (abs(point.z - closest_z - height) <= 0.001)
			{
				float radius = (point.z - cone_vertex[2]) * tan(slope);
				float distance = std::sqrt(point.x * point.x + (point.y - cone_vertex[1]) * (point.y - cone_vertex[1]));

				float dis = radius - distance;

				if (dis <= 3.378 && dis >= 0)					
				{
					cloud_cone->points.push_back(point);
				}
			}
			else
			{
				// 计算点Z值对应的半径，然后通过与半径的比较进行计算 
				double radius = (point.z - cone_vertex[2]) * tan(slope);
				double distance = std::sqrt(point.x * point.x + (point.y - cone_vertex[1]) * (point.y - cone_vertex[1]));

				if (abs(distance - radius) <= 4.078)
				{
					cloud_cone->points.push_back(point);
				}
			}

			/*cloud_cone->points.push_back(point);*/
		}
	}

	// 打印一些结果（可以根据需要输出或处理这些距离）
	//for (size_t i = 0; i < distances.size(); ++i) {
	//	std::cout << "Point " << i << " distance to cone: " << distances[i] << std::endl;
	//}
	auto endOp = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp = endOp - startOp;
	std::cout << "圆锥抽取算法用时: " << elapsedOp.count() << " seconds" << std::endl;

	std::cout << "Extracted " << cloud_cone->points.size() << " points within the cone" << std::endl;
	// 保存用
	//cloud_cone->width = cloud_cone->points.size();
	//cloud_cone->height = 1;
	//cloud_cone->is_dense = false;

	//pcl::PCDWriter writer;
	//writer.write("ExtractedCone.pcd", *cloud_cone, false);

	// 4. 显示筛选后的点云
	pcl::visualization::PCLVisualizer viewer("ConeViewer");
	viewer.addPointCloud(cloud_cone);
	viewer.resetCamera();

	// 等待直到视图关闭
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}

	return 0;
}