#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <cmath>
#include <pcl/visualization/cloud_viewer.h>

// �����������
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

const int numLines = 360; // ����ĸ�ߵķֱ���

// ����㵽�߶ε���̾��뺯��
double pointToLineDistance(const pcl::PointXYZ& point,
	const Eigen::Vector3d& linePoint,
	const Eigen::Vector3d& lineDirection) {
	Eigen::Vector3d pointVec(point.x - linePoint.x(), point.y - linePoint.y(), point.z - linePoint.z());

	// ����㵽�߶ε�ͶӰ
	Eigen::Vector3d projection = lineDirection * (pointVec.dot(lineDirection) / lineDirection.squaredNorm());

	// �㵽�ߵ�����
	Eigen::Vector3d perpendicular = pointVec - projection;

	return perpendicular.norm();
}

// ����㵽Բ׶ĸ�ߵ���̾��뺯��
double pointToConeShortestDistance(const pcl::PointXYZ& point,
	const Eigen::Vector3d& coneVertex,
	const Eigen::Vector3d& coneAxis,
	double coneSlope) {
	double minDistance = std::numeric_limits<double>::max(); // ��ʼ��Ϊһ���ܴ��ֵ

	// �������е�ĸ�߷���
	for (int i = 0; i < numLines; ++i) {
		// ���㵱ǰĸ�ߵ���ת�Ƕ�
		double angle = i * 2 * M_PI / numLines;

		// ����ĸ�ߵķ�������
		Eigen::Vector3d rotationAxis(0, 0, 1);  // Z��
		Eigen::AngleAxisd rotation(angle, rotationAxis);  // ��Z����ת�ĽǶ�
		Eigen::Vector3d lineDirection = rotation * coneAxis; // �õ���ת�������

		// ����Ϊĸ�߷��򣬿���׶��
		Eigen::Vector3d coneDirection = lineDirection * tan(coneSlope);
		coneDirection.normalize(); // ��׼������

		// ����㵽��ǰĸ�ߵ���̾���
		double distance = pointToLineDistance(point, coneVertex, coneDirection);

		// ������С����
		if (distance < minDistance) {
			minDistance = distance;
		}
	}

	return minDistance;
}

// ����㵽Բ׶��ľ���
double pointToConeDistance(const pcl::PointXYZ& point,
	const Eigen::Vector3d& vertex,
	const Eigen::Vector3d& axis,
	double slope,
	double coneHeight,
	double coneRadius)
{
	// �������Բ׶���������
	Eigen::Vector3d pointVec(point.x - vertex.x(), point.y - vertex.y(), point.z - vertex.z());

	// ����ͶӰ��Բ׶�����ϣ�����ͶӰ�ĸ߶�
	double heightAlongAxis = pointVec.dot(axis);
	if (heightAlongAxis < 0)
	{
		// If the point is below the cone (outside), return distance to vertex
		double distanceToVertex = pointVec.norm();
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
	Eigen::Vector3d axisProjection = heightAlongAxis * axis;
	Eigen::Vector3d radialDirection = pointVec - axisProjection;

	// ����㵽Բ׶����ľ���
	double radialDistance = radialDirection.norm();
	double distanceToSurface = std::abs(radialDistance - radiusAtHeight) * cos(slope);
	return distanceToSurface;
}

int main(int argc, char** argv)
{
	// ���������ص���
	PointCloud::Ptr cloud(new PointCloud);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("ConeExtract.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file input_cloud.pcd\n");
		return (-1);
	}

	// Բ׶����
	Eigen::Vector3d coneVertex(0.342, 0.603, 62.160);  // ��������
	Eigen::Vector3d coneAxis(0.0f, 0.0f, 1.0f); // ���򣨼���Z�᷽��
	double coneSlope = 24.575f * M_PI / 180.0f; // Բ׶�¶�
	double coneHeight = 62.160f; // Բ׶�߶�
	double coneRadius = coneHeight * tan(coneSlope); // Բ׶����뾶
	double ballRadius = 4.0f; // ������İ뾶

	// ��ȡ���緶Χ�ڵĵ���
	PointCloud::Ptr extractedCloud(new PointCloud);

	for (const auto& point : cloud->points)
	{
		// ����㵽Բ׶����ľ���
		double distance = pointToConeDistance(point, coneVertex, coneAxis, coneSlope, coneHeight, coneRadius);
		//double distance = pointToConeShortestDistance(point, coneVertex, coneAxis, coneSlope);
		// �������С�ڹ�����İ뾶���򽫵���뵽��ȡ�ĵ���
		if (distance <= ballRadius)
		{
			extractedCloud->points.push_back(point);
		}
	}

	std::cout << "Extracted " << extractedCloud->points.size() << " points within the cone" << std::endl;

	pcl::visualization::PCLVisualizer viewer("ConeViewer");
	viewer.addPointCloud(extractedCloud);
	viewer.resetCamera();

	// �ȴ�ֱ����ͼ�ر�
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
	return 0;
}