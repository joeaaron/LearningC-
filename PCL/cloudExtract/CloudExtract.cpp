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

// 计算点到圆锥母线的垂足点
Eigen::Vector3f pointToConeLineFoot(const pcl::PointXYZ& P,
	const Eigen::Vector3f& vertex,
	const Eigen::Vector3f& axis,
	double slope)
{
	// 1. 计算点P到圆锥顶点的向量
	Eigen::Vector3f pointVec(P.x - vertex.x(), P.y - vertex.y(), P.z - vertex.z());

	// 2. 投影点P到圆锥轴线，得到投影点Q
	double heightAlongAxis = pointVec.dot(axis);  // 点P在轴线上的投影长度
	Eigen::Vector3f Q = vertex + heightAlongAxis * axis;  // 投影点Q

	// 3. 计算圆锥在该高度下的半径
	double radiusAtHeight = heightAlongAxis * tan(slope);

	// 4. 计算径向向量
	Eigen::Vector3f radialVec = pointVec - (heightAlongAxis * axis);

	// 如果径向向量长度为零，说明点P在圆锥的轴线上
	if (radialVec.norm() == 0) {
		return Q;  // 返回投影点Q作为垂足点
	}

	// 5. 归一化径向向量，找到该高度下圆锥表面上的点Q'
	Eigen::Vector3f radialVecNormalized = radialVec.normalized();
	Eigen::Vector3f surfacePoint = Q + radiusAtHeight * radialVecNormalized;  // 圆锥表面点Q'

	// 6. 使用点到线的垂足公式，计算点P到圆锥母线的垂足点
	// 计算母线的方向向量
	Eigen::Vector3f coneLineDirection = surfacePoint - vertex;  // 母线方向
	Eigen::Vector3f PtoVertex = Eigen::Vector3f(P.x, P.y, P.z) - vertex;  // 点P到圆锥顶点的向量

	// 投影系数t
	double t = PtoVertex.dot(coneLineDirection) / coneLineDirection.dot(coneLineDirection);

	// 计算垂足点F
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
	// 点相对于圆锥顶点的向量
	Eigen::Vector3f pointVec(point.x - vertex.x(), point.y - vertex.y(), point.z - vertex.z());

	Eigen::Vector3f footPoint;  // 垂足

	// 将点投影到圆锥轴线上，计算投影的高度
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

	// 计算该高度处的圆锥半径
	double radiusAtHeight = heightAlongAxis * tan(slope);

	// 计算点到轴线的径向向量
	Eigen::Vector3f axisProjection = heightAlongAxis * axis;
	Eigen::Vector3f radialDirection = pointVec - axisProjection;

	// 计算点到圆锥表面的距离
	double radialDistance = radialDirection.norm();

	// 计算垂足在圆锥表面的位置
	if (radialDistance > 0)
	{
		// 归一化径向方向
		Eigen::Vector3f radialDirectionNormalized = radialDirection.normalized();

		// 对应半径方向上的点
		Eigen::Vector3f radiusPt = radiusAtHeight * radialDirectionNormalized;

		Eigen::Vector3f radiusVec(point.x - radiusPt.x(), point.y - radiusPt.y(), point.z - radiusPt.z());
		double r = std::abs(radialDistance - radiusAtHeight) * sin(slope);
		
		footPoint = r * radiusVec.normalized();
	}
	else
	{
		// 如果径向距离为0，垂足点在轴线上
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
	// 1. 读取点云数据---对应PWK圆锥抽取-4
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT>("cube3.pcd", *cloud) == -1) {
		PCL_ERROR("Couldn't read file input.pcd \n");
		return -1;
	}
	std::cout << "Loaded " << cloud->width * cloud->height << " data points"  << std::endl;

	auto startOp = std::chrono::high_resolution_clock::now();

	// 2. 设置圆锥参数
	Eigen::Vector3f cone_vertex(0.000, 101.99999999, -55.98076197);   // 圆锥顶点位置
	Eigen::Vector3f cone_axis(0.000, 0.000, 1);			// 圆锥轴向方向
	cone_axis.normalize();								    // 规范化轴向方向

	double slope = M_PI / 6.0 * 0.5;						// 圆锥的坡度（弧度），即30度
	double apexDistance = 30.98076196;							// 从顶点的距离
	double height = 25.00000002;								    // 圆锥的高度
	
	// 3. 筛选点云中的候选点
	pcl::PointCloud<PointT>::Ptr cloud_cone(new pcl::PointCloud<PointT>);
	std::vector<double> distances;

	// 找到离从顶点最近的Z值 [-25.310, 0.690]
	double closest_z = findClosestZValueKDTree(cloud, cone_vertex[2] + apexDistance);

	for (const auto& point : cloud->points)  
	{
		// 单独测试
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
			//// 顶部点要特殊筛选
			//if (abs(point.z - closest_z) <= 0.001)
			//{
			//	double radius = (point.z - cone_vertex[2]) * tan(slope);
			//	double distance = std::sqrt(point.x * point.x + (point.y - cone_vertex[1]) * (point.y - cone_vertex[1]));

			//	if (((distance - radius) <= 4.078) && ((distance - radius) >= 3.102))  
			//	{
			//		cloud_cone->points.push_back(point);
			//	}
			//}
			//// 底部点要特殊筛选
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
			//	// 计算点Z值对应的半径，然后通过与半径的比较进行计算 
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

	// 打印一些结果（可以根据需要输出或处理这些距离）
	//for (size_t i = 0; i < distances.size(); ++i) {
	//	std::cout << "Point " << i << " distance to cone: " << distances[i] << std::endl;
	//}
	auto endOp = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp = endOp - startOp;
	std::cout << "圆锥抽取算法用时: " << elapsedOp.count() << " seconds" << std::endl;

	std::cout << "Extracted " << cloud_cone->points.size() << " points within the cone" << std::endl;
	// 保存用
	cloud_cone->width = cloud_cone->points.size();
	cloud_cone->height = 1;
	cloud_cone->is_dense = false;

	pcl::PCDWriter writer;
	writer.write("ExtractedCone11.pcd", *cloud_cone, false);

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
