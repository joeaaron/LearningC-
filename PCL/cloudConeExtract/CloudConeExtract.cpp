#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <cmath>
#include <pcl/visualization/cloud_viewer.h>

// 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

const int numLines = 360; // 遍历母线的分辨率

// 计算点到线段的最短距离函数
double pointToLineDistance(const pcl::PointXYZ& point,
	const Eigen::Vector3d& linePoint,
	const Eigen::Vector3d& lineDirection) {
	Eigen::Vector3d pointVec(point.x - linePoint.x(), point.y - linePoint.y(), point.z - linePoint.z());

	// 计算点到线段的投影
	Eigen::Vector3d projection = lineDirection * (pointVec.dot(lineDirection) / lineDirection.squaredNorm());

	// 点到线的向量
	Eigen::Vector3d perpendicular = pointVec - projection;

	return perpendicular.norm();
}

// 计算点到圆锥母线的最短距离函数
double pointToConeShortestDistance(const pcl::PointXYZ& point,
	const Eigen::Vector3d& coneVertex,
	const Eigen::Vector3d& coneAxis,
	double coneSlope) {
	double minDistance = std::numeric_limits<double>::max(); // 初始化为一个很大的值

	// 遍历所有的母线方向
	for (int i = 0; i < numLines; ++i) {
		// 计算当前母线的旋转角度
		double angle = i * 2 * M_PI / numLines;

		// 计算母线的方向向量
		Eigen::Vector3d rotationAxis(0, 0, 1);  // Z轴
		Eigen::AngleAxisd rotation(angle, rotationAxis);  // 绕Z轴旋转的角度
		Eigen::Vector3d lineDirection = rotation * coneAxis; // 得到旋转后的轴向

		// 调整为母线方向，考虑锥角
		Eigen::Vector3d coneDirection = lineDirection * tan(coneSlope);
		coneDirection.normalize(); // 标准化方向

		// 计算点到当前母线的最短距离
		double distance = pointToLineDistance(point, coneVertex, coneDirection);

		// 更新最小距离
		if (distance < minDistance) {
			minDistance = distance;
		}
	}

	return minDistance;
}

// 计算点到圆锥轴的距离
double pointToConeDistance(const pcl::PointXYZ& point,
	const Eigen::Vector3d& vertex,
	const Eigen::Vector3d& axis,
	double slope,
	double coneHeight,
	double coneRadius)
{
	// 点相对于圆锥顶点的向量
	Eigen::Vector3d pointVec(point.x - vertex.x(), point.y - vertex.y(), point.z - vertex.z());

	// 将点投影到圆锥轴线上，计算投影的高度
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

	// 计算该高度处的圆锥半径
	double radiusAtHeight = heightAlongAxis * tan(slope);

	// 计算点到轴线的径向向量
	Eigen::Vector3d axisProjection = heightAlongAxis * axis;
	Eigen::Vector3d radialDirection = pointVec - axisProjection;

	// 计算点到圆锥表面的距离
	double radialDistance = radialDirection.norm();
	double distanceToSurface = std::abs(radialDistance - radiusAtHeight) * cos(slope);
	return distanceToSurface;
}

int main(int argc, char** argv)
{
	// 创建并加载点云
	PointCloud::Ptr cloud(new PointCloud);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("ConeExtract.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file input_cloud.pcd\n");
		return (-1);
	}

	// 圆锥参数
	Eigen::Vector3d coneVertex(0.342, 0.603, 62.160);  // 顶点坐标
	Eigen::Vector3d coneAxis(0.0f, 0.0f, 1.0f); // 轴向（假设Z轴方向）
	double coneSlope = 24.575f * M_PI / 180.0f; // 圆锥坡度
	double coneHeight = 62.160f; // 圆锥高度
	double coneRadius = coneHeight * tan(coneSlope); // 圆锥底面半径
	double ballRadius = 4.0f; // 滚动球的半径

	// 提取包络范围内的点云
	PointCloud::Ptr extractedCloud(new PointCloud);

	for (const auto& point : cloud->points)
	{
		// 计算点到圆锥表面的距离
		double distance = pointToConeDistance(point, coneVertex, coneAxis, coneSlope, coneHeight, coneRadius);
		//double distance = pointToConeShortestDistance(point, coneVertex, coneAxis, coneSlope);
		// 如果距离小于滚动球的半径，则将点加入到提取的点云
		if (distance <= ballRadius)
		{
			extractedCloud->points.push_back(point);
		}
	}

	std::cout << "Extracted " << extractedCloud->points.size() << " points within the cone" << std::endl;

	pcl::visualization::PCLVisualizer viewer("ConeViewer");
	viewer.addPointCloud(extractedCloud);
	viewer.resetCamera();

	// 等待直到视图关闭
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
	return 0;
}