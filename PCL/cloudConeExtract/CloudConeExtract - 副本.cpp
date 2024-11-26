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
	auto distance = point.z - apexDistance;  // 浮点型相减有精度损失
	return distance >= 0 && ((distance - height) <= 0.01);
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

	// 2. 设置圆锥参数
	//Eigen::Vector3f cone_vertex(0.000, 101.99999999, -55.98076197); // 圆锥顶点位置
	//Eigen::Vector3f cone_axis(0.000, 0.000, -1);					  // 圆锥轴向方向
	//cone_axis.normalize();										  // 规范化轴向方向

	//double slope = M_PI / 6.0 * 0.5;								  // 圆锥的坡度（弧度），即30度
	//double apexDistance = 30.98076196;							  // 从顶点的距离
	//double height = 25.00000002;									  // 圆锥的高度
	
	Eigen::Vector3f cone_vertex(0.342, 0.603, 62.160);				  // 圆锥顶点位置
	Eigen::Vector3f cone_axis(0.000, 0.000, -1);					  // 圆锥轴向方向
	cone_axis.normalize();											  // 规范化轴向方向

	double slope = 24.575 * M_PI / 180;								  // 圆锥的坡度（弧度），即30度
	double apexDistance = 0;										  // 从顶点的距离
	double height = 62.160;											  // 圆锥的高度

	// TEST
	//Eigen::Vector3f pt(11.486, 104.271, 0.880);  
	//Eigen::Vector3f pt(11.820, 104.044, -25.310);
	//Eigen::Vector3f pt(-0.214, 113.571, 0.880);
	//Eigen::Vector3f pt(12.086, 104.271, -25.520);					//31.73
	//Eigen::Vector3f pt(0.686, 114.171, -25.520);	
	//Eigen::Vector3f pt(1.097, 1.192, 65.927);	  //3.53
	//Eigen::Vector3f pt(3.097, 0.192, 64.927);	  //3.57
	Eigen::Vector3f pt0(-1.903, 0.808, 65.927);
	Eigen::Vector3f pt1(-0.903, 0.192, 65.927);	  
	Eigen::Vector3f pt2(0.097, 0.192, 65.927);
	Eigen::Vector3f pt3(0.097, 1.192, 65.927);
	Eigen::Vector3f pt4(1.097, 0.192, 65.927);
	Eigen::Vector3f pt5(1.097, 1.192, 65.927);
	Eigen::Vector3f pt6(-2.903486, -0.807786, 63.926884);

	Eigen::Vector3f dis0 = pt0 - cone_vertex;
	auto length0 = (dis0.norm()) * cos(slope);
	auto width0 = (dis0.norm());  //std::sqrt(dis1[0] * dis1[0] + dis1[1] * dis1[1]); /*(dis.norm()) * sin(slope);	*/				// 15.01
	double radius0 = (pt0[2] - cone_vertex[2]) * tan(slope);

	Eigen::Vector3f dis1 = pt1 - cone_vertex;
	auto length1    = (dis1.norm()) * cos(slope);					 
	auto width1 = (dis1.norm());  //std::sqrt(dis1[0] * dis1[0] + dis1[1] * dis1[1]); /*(dis.norm()) * sin(slope);	*/				// 15.01
	double radius1 = (pt1[2] - cone_vertex[2]) * tan(slope); 

	Eigen::Vector3f dis2 = pt2 - cone_vertex;
	auto length2 = (dis2.norm()) * cos(slope);
	auto width2 = (dis2.norm());  //std::sqrt(dis2[0] * dis2[0] + dis2[1] * dis2[1]); /*(dis.norm()) * sin(slope);	*/				// 15.01
	double radius2 = (pt2[2] - cone_vertex[2]) * tan(slope);

	Eigen::Vector3f dis3 = pt3 - cone_vertex;
	auto length3 = (dis3.norm()) * cos(slope);
	auto width3 = (dis3.norm());  //std::sqrt(dis3[0] * dis3[0] + dis3[1] * dis3[1]); /*(dis.norm()) * sin(slope);	*/				// 15.01
	double radius3 = (pt3[2] - cone_vertex[2]) * tan(slope);

	Eigen::Vector3f dis4 = pt4 - cone_vertex;
	auto length4 = (dis4.norm()) * cos(slope);
	auto width4 = (dis4.norm());//std::sqrt(dis4[0] * dis4[0] + dis4[1] * dis4[1]); /*(dis.norm()) * sin(slope);	*/				// 15.01
	double radius4 = (pt4[2] - cone_vertex[2]) * tan(slope);

	Eigen::Vector3f dis5 = pt5 - cone_vertex;
	auto length5 = (dis5.norm()) * cos(slope);
	auto width5 = (dis5.norm());  //std::sqrt(dis5[0] * dis5[0] + dis5[1] * dis5[1]); /*(dis.norm()) * sin(slope);	*/				// 15.01
	double radius5 = (pt5[2] - cone_vertex[2]) * tan(slope);

	Eigen::Vector3f dis6 = pt6 - cone_vertex;
	auto length6 = (dis6.norm()) * cos(slope);
	auto width6 = (dis6.norm());  //std::sqrt(dis5[0] * dis5[0] + dis5[1] * dis5[1]); /*(dis.norm()) * sin(slope);	*/				// 15.01
	double radius6 = (pt6[2] - cone_vertex[2]) * tan(slope);

	auto startOp = std::chrono::high_resolution_clock::now();
	
	cout << "length0:" << length0 << " " << "width0:" << width0 << " " << "radius0:" << radius0 << endl;
	cout << "length1:" << length1 << " " << "width1:" << width1 << " " << "radius1:" << radius1 << endl;
	cout << "length2:" << length2 << " " << "width2:" << width2 << " " << "radius2:" << radius2 << endl;
	cout << "length3:" << length3 << " " << "width3:" << width3 << " " << "radius3:" << radius3 << endl;
	cout << "length4:" << length4 << " " << "width4:" << width4 << " " << "radius4:" << radius4 << endl;
	cout << "length5:" << length5 << " " << "width5:" << width5 << " " << "radius5:" << radius5 << endl;
	cout << "length6:" << length6 << " " << "width5:" << width5 << " " << "radius6:" << radius6 << endl;
	return 0;
}