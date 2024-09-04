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

// ����㵽Բ׶��ľ���
double distanceToConeAxis(const PointT& point, const Eigen::Vector3f& cone_vertex, const Eigen::Vector3f& cone_axis) 
{
	// ��һ�� cone_axis ȷ�����ǵ�λ����
	Eigen::Vector3f normalized_axis = cone_axis.normalized();

	Eigen::Vector3f point_vec = point.getVector3fMap() - cone_vertex;
	Eigen::Vector3f projection = point_vec.dot(normalized_axis) * normalized_axis;
	double distance = (point_vec - projection).norm();
	return distance;
}

// �жϵ��Ƿ���Բ׶�ĸ߶ȷ�Χ��(������н�Ϊ������������ϵ�ͶӰ����<=height)
bool WithinConeHeight(const PointT& point, 
	const Eigen::Vector3f& cone_vertex, 
	const Eigen::Vector3f& cone_axis, 
	double apexDistance, double height)
{
	//Eigen::Vector3f point_vec = point.getVector3fMap() - cone_vertex;
	//double distance_along_axis = point_vec.dot(cone_axis) + apexDistance;				// ����Բ׶�᷽���ϵ�ͶӰ����
	auto distance = point.z - apexDistance;  // ����������о�����ʧ
	return distance >= 0 && ((distance - height) <= 0.01);
}

// ����㵽Բ׶�����̾���
double ShortestDistanceToCone(const PointT& point, 
	const Eigen::Vector3f& cone_vertex,
	const Eigen::Vector3f& cone_axis,
	double slope) 
{
	Eigen::Vector3f point_vec = point.getVector3fMap() - cone_vertex;
	double distance_along_axis = point_vec.dot(cone_axis);  

	double radius_at_point = distance_along_axis * std::tan(slope);					// ��ǰ�߶ȵ�Բ׶�뾶
	double distance_to_axis = distanceToConeAxis(point, cone_vertex, cone_axis);

	return std::abs(distance_to_axis - radius_at_point);
}

double findClosestZValueKDTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double target_z) {
	// ���� KD-Tree ���󲢽���������
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	// ����������
	pcl::PointXYZ searchPoint;
	searchPoint.z = target_z;
	searchPoint.x = 0.0f;  // X �� Y ��ֵ�������޹ؽ�Ҫ
	searchPoint.y = 0.0f;

	// ������ӽ���һ����
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
	// 1. ��ȡ��������
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT>("cube3.pcd", *cloud) == -1) {
		PCL_ERROR("Couldn't read file input.pcd \n");
		return -1;
	}
	std::cout << "Loaded " << cloud->width * cloud->height << " data points"  << std::endl;

	auto startOp = std::chrono::high_resolution_clock::now();

	// 2. ����Բ׶����
	Eigen::Vector3f cone_vertex(0.000, 101.99999999, -55.98076197);   // Բ׶����λ��
	Eigen::Vector3f cone_axis(0.000, 0.000, -1);			// Բ׶������
	cone_axis.normalize();								    // �淶��������

	double slope = M_PI / 6.0 * 0.5;						// Բ׶���¶ȣ����ȣ�����30��
	double apexDistance = 30.98076196;							// �Ӷ���ľ���
	double height = 25.00000002;								    // Բ׶�ĸ߶�
	
	// 3. ɸѡ�����еĺ�ѡ��
	pcl::PointCloud<PointT>::Ptr cloud_cone(new pcl::PointCloud<PointT>);
	std::vector<double> distances;

	// �ҵ���Ӷ��������Zֵ
	double closest_z = findClosestZValueKDTree(cloud, cone_vertex[2] + apexDistance);

	for (const auto& point : cloud->points)
	{
		if (WithinConeHeight(point, cone_vertex, cone_axis, closest_z, height))
		{
			// ������Ҫ����ɸѡ
			if (abs(point.z - closest_z) <= 0.001)
			{
				double radius = (point.z - cone_vertex[2]) * tan(slope);
				double distance = std::sqrt(point.x * point.x + (point.y - cone_vertex[1]) * (point.y - cone_vertex[1]));

				if (((distance - radius) <= 4.078) && ((distance - radius) >= 3.102))  
				{
					cloud_cone->points.push_back(point);
				}
			}
			// �ײ���Ҫ����ɸѡ
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
				// �����Zֵ��Ӧ�İ뾶��Ȼ��ͨ����뾶�ıȽϽ��м��� 
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

	// ��ӡһЩ��������Ը�����Ҫ���������Щ���룩
	//for (size_t i = 0; i < distances.size(); ++i) {
	//	std::cout << "Point " << i << " distance to cone: " << distances[i] << std::endl;
	//}
	auto endOp = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp = endOp - startOp;
	std::cout << "Բ׶��ȡ�㷨��ʱ: " << elapsedOp.count() << " seconds" << std::endl;

	std::cout << "Extracted " << cloud_cone->points.size() << " points within the cone" << std::endl;
	// ������
	//cloud_cone->width = cloud_cone->points.size();
	//cloud_cone->height = 1;
	//cloud_cone->is_dense = false;

	//pcl::PCDWriter writer;
	//writer.write("ExtractedCone.pcd", *cloud_cone, false);

	// 4. ��ʾɸѡ��ĵ���
	pcl::visualization::PCLVisualizer viewer("ConeViewer");
	viewer.addPointCloud(cloud_cone);
	viewer.resetCamera();

	// �ȴ�ֱ����ͼ�ر�
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}

	return 0;
}