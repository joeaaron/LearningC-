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

// ����㵽Բ׶ĸ�ߵĴ����
Eigen::Vector3f pointToConeLineFoot(const pcl::PointXYZ& P,
	const Eigen::Vector3f& vertex,
	const Eigen::Vector3f& axis,
	double slope)
{
	// 1. �����P��Բ׶���������
	Eigen::Vector3f pointVec(P.x - vertex.x(), P.y - vertex.y(), P.z - vertex.z());

	// 2. ͶӰ��P��Բ׶���ߣ��õ�ͶӰ��Q
	double heightAlongAxis = pointVec.dot(axis);  // ��P�������ϵ�ͶӰ����
	Eigen::Vector3f Q = vertex + heightAlongAxis * axis;  // ͶӰ��Q

	// 3. ����Բ׶�ڸø߶��µİ뾶
	double radiusAtHeight = heightAlongAxis * tan(slope);

	// 4. ���㾶������
	Eigen::Vector3f radialVec = pointVec - (heightAlongAxis * axis);

	// ���������������Ϊ�㣬˵����P��Բ׶��������
	if (radialVec.norm() == 0) {
		return Q;  // ����ͶӰ��Q��Ϊ�����
	}

	// 5. ��һ�������������ҵ��ø߶���Բ׶�����ϵĵ�Q'
	Eigen::Vector3f radialVecNormalized = radialVec.normalized();
	Eigen::Vector3f surfacePoint = Q + radiusAtHeight * radialVecNormalized;  // Բ׶�����Q'

	// 6. ʹ�õ㵽�ߵĴ��㹫ʽ�������P��Բ׶ĸ�ߵĴ����
	// ����ĸ�ߵķ�������
	Eigen::Vector3f coneLineDirection = surfacePoint - vertex;  // ĸ�߷���
	Eigen::Vector3f PtoVertex = Eigen::Vector3f(P.x, P.y, P.z) - vertex;  // ��P��Բ׶���������

	// ͶӰϵ��t
	double t = PtoVertex.dot(coneLineDirection) / coneLineDirection.dot(coneLineDirection);

	// ���㴹���F
	Eigen::Vector3f footPoint = vertex + t * coneLineDirection;

	return footPoint;
}

double pointToConeDistance(const pcl::PointXYZ& point,
	const Eigen::Vector3f& vertex,
	const Eigen::Vector3f& axis,
	double slope,
	double coneHeight,
	double coneRadius)
{
	// �������Բ׶���������
	Eigen::Vector3f pointVec(point.x - vertex.x(), point.y - vertex.y(), point.z - vertex.z());

	Eigen::Vector3f footPoint;  // ����

	// ����ͶӰ��Բ׶�����ϣ�����ͶӰ�ĸ߶�
	double heightAlongAxis = pointVec.dot(axis);
	if (heightAlongAxis < 0)
	{
		// If the point is below the cone (outside), return distance to vertex
		double distanceToVertex = pointVec.norm();
		footPoint = vertex; // The foot point is the vertex
		return distanceToVertex <= 4 ? distanceToVertex : std::numeric_limits<double>::max();
	}
	//else if (heightAlongAxis > coneHeight)
	//{
	//	// If the point is above the cone, return the distance to the cone's base
	//	Eigen::Vector3d coneBase = vertex + coneHeight * axis;
	//	double distanceToBase = (pointVec - coneBase).norm();
	//	return distanceToBase;
	//}

	// ����ø߶ȴ���Բ׶�뾶
	double radiusAtHeight = heightAlongAxis * tan(slope);

	// ����㵽���ߵľ�������
	Eigen::Vector3f axisProjection = heightAlongAxis * axis;
	Eigen::Vector3f radialDirection = pointVec - axisProjection;

	// ����㵽Բ׶����ľ���
	double radialDistance = radialDirection.norm();

	// ���㴹����Բ׶�����λ��
	if (radialDistance > 0)
	{
		// ��һ��������
		Eigen::Vector3f radialDirectionNormalized = radialDirection.normalized();

		// ��Ӧ�뾶�����ϵĵ�
		Eigen::Vector3f radiusPt = radiusAtHeight * radialDirectionNormalized;

		Eigen::Vector3f radiusVec(point.x - radiusPt.x(), point.y - radiusPt.y(), point.z - radiusPt.z());
		double r = std::abs(radialDistance - radiusAtHeight) * sin(slope);
		
		footPoint = r * radiusVec.normalized();
	}
	else
	{
		// ����������Ϊ0���������������
		//footPoint = vertex + axisProjection;
	}

	if (footPoint[2] < -25 || footPoint[2] > 0)
	{
		return std::numeric_limits<double>::max();
	}
	double distanceToSurface = std::abs(radialDistance - radiusAtHeight) * cos(slope);
	return distanceToSurface;
}

int main(int argc, char** argv) 
{
	// 1. ��ȡ��������---��ӦPWKԲ׶��ȡ-4
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT>("cube3.pcd", *cloud) == -1) {
		PCL_ERROR("Couldn't read file input.pcd \n");
		return -1;
	}
	std::cout << "Loaded " << cloud->width * cloud->height << " data points"  << std::endl;

	auto startOp = std::chrono::high_resolution_clock::now();

	// 2. ����Բ׶����
	Eigen::Vector3f cone_vertex(0.000, 101.99999999, -55.98076197);   // Բ׶����λ��
	Eigen::Vector3f cone_axis(0.000, 0.000, 1);			// Բ׶������
	cone_axis.normalize();								    // �淶��������

	double slope = M_PI / 6.0 * 0.5;						// Բ׶���¶ȣ����ȣ�����30��
	double apexDistance = 30.98076196;							// �Ӷ���ľ���
	double height = 25.00000002;								    // Բ׶�ĸ߶�
	
	// 3. ɸѡ�����еĺ�ѡ��
	pcl::PointCloud<PointT>::Ptr cloud_cone(new pcl::PointCloud<PointT>);
	std::vector<double> distances;

	// �ҵ���Ӷ��������Zֵ [-25.310, 0.690]
	double closest_z = findClosestZValueKDTree(cloud, cone_vertex[2] + apexDistance);

	for (const auto& point : cloud->points)  
	{
		// ��������
		//pcl::PointXYZ pt(-16.179623, 97.033104, -0.309795);
		//pcl::PointXYZ pt(-11.179623, 97.033104, 0.690);
		//pcl::PointXYZ pt(-12.179623, 100.033104, 0.690);
		//pcl::PointXYZ pt(-16.180, 93.033104, -1.309795);
		Eigen::Vector3f foot = pointToConeLineFoot(point, cone_vertex, cone_axis, slope);
		double dZ = foot[2];
		if (dZ < -25 || dZ > 0)
		{
			continue;
		}

		Eigen::Vector3f pointFoot(point.x - foot.x(), point.y - foot.y(), point.z - foot.z());
		double distance = pointFoot.norm();
		if (distance <= 4)
		{
			cloud_cone->points.push_back(point);
		}
	
		if (WithinConeHeight(point, cone_vertex, cone_axis, closest_z, height+1))
		{
		/*	double distance = pointToConeDistance(point, cone_vertex, cone_axis, slope, height, 4);
			if (distance <= 4)
			{
				cloud_cone->points.push_back(point);
			}*/
			//// ������Ҫ����ɸѡ
			//if (abs(point.z - closest_z) <= 0.001)
			//{
			//	double radius = (point.z - cone_vertex[2]) * tan(slope);
			//	double distance = std::sqrt(point.x * point.x + (point.y - cone_vertex[1]) * (point.y - cone_vertex[1]));

			//	if (((distance - radius) <= 4.078) && ((distance - radius) >= 3.102))  
			//	{
			//		cloud_cone->points.push_back(point);
			//	}
			//}
			//// �ײ���Ҫ����ɸѡ
			//else if (abs(point.z - closest_z - height) <= 0.001)
			//{
			//	float radius = (point.z - cone_vertex[2]) * tan(slope);
			//	float distance = std::sqrt(point.x * point.x + (point.y - cone_vertex[1]) * (point.y - cone_vertex[1]));

			//	float dis = radius - distance;

			//	if (dis <= 3.378 && dis >= 0)					
			//	{
			//		cloud_cone->points.push_back(point);
			//	}
			//}
			//else
			//{
			//	// �����Zֵ��Ӧ�İ뾶��Ȼ��ͨ����뾶�ıȽϽ��м��� 
			//	double radius = (point.z - cone_vertex[2]) * tan(slope);
			//	double distance = std::sqrt(point.x * point.x + (point.y - cone_vertex[1]) * (point.y - cone_vertex[1]));

			//	if (abs(distance - radius) <= 4.078)
			//	{
			//		cloud_cone->points.push_back(point);
			//	}
			//}

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
	cloud_cone->width = cloud_cone->points.size();
	cloud_cone->height = 1;
	cloud_cone->is_dense = false;

	pcl::PCDWriter writer;
	writer.write("ExtractedCone11.pcd", *cloud_cone, false);

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
