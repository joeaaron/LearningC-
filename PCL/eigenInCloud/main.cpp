#define EIGEN_USE_MKL_ALL
#define EIGEN_VECTORIZE_SSE4_2

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <chrono>

int main(int argc, char** argv)
{
	Eigen::MatrixXd m1 = Eigen::MatrixXd::Random(10000, 10000);
	Eigen::MatrixXd m2 = Eigen::MatrixXd::Random(10000, 10000);

	auto start = std::chrono::high_resolution_clock::now();
	Eigen::MatrixXd p = m1 * m2;
	auto end = std::chrono::high_resolution_clock::now();

	std::chrono::duration<double, std::milli> duration = end - start;
	std::cout << "Eigen矩阵相乘用时： " << duration.count() << "ms" << std::endl;
	//std::vector<Eigen::Vector4d> vec3DPoints;
	//vec3DPoints.push_back(Eigen::Vector4d(1.0, 2.0, 3.0, 1));
	//vec3DPoints.push_back(Eigen::Vector4d(4.0, 5.0, 6.0, 1));
	//vec3DPoints.push_back(Eigen::Vector4d(1.0, 2.0, 3.0, 1));
	//vec3DPoints.push_back(Eigen::Vector4d(4.0, 5.0, 6.0, 1));

	//Eigen::Map<Eigen::MatrixXd> aMx(vec3DPoints.data()->data(), 4, vec3DPoints.size());
	//std::cout << "3D matrix:" << std::endl;
	//std::cout << aMx << std::endl;


	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	//for (size_t i = 0; i < 10; i++)
	//{
	//	pcl::PointXYZ p;
	//	p.x = i + 0.1;
	//	p.y = i + 0.2;
	//	p.z = i + 0.3;
	//	cloudPtr->points.emplace_back(p);
	//}

	//// 点云转矩阵
	//Eigen::Map<Eigen::MatrixXf> eigen_map(reinterpret_cast<float*>(cloudPtr->points.data()), 4, cloudPtr->points.size());  // 创建一个 Eigen::Map 对象，映射到点云数据的内存

	//std::cout << "Point cloud matrix:" << std::endl;
	//std::cout << eigen_map << std::endl;

	//Eigen::MatrixXf normalMx = eigen_map.topRows(3).array().rowwise() /
	//	eigen_map.row(3).array();
	//for (size_t i = 0; i < normalMx.cols(); ++i)
	//{
	//	bool bInFrustum = std::abs(normalMx.col(i)[0]) <= 1.0 && std::abs(normalMx.col(i)[1]) <= 1.0 && std::abs(normalMx.col(2)[0]) <= 1.0;
	//	normalMx.col(i)[2] = bInFrustum;
	//}
	///*Eigen::Matrix<double, 4, Eigen::Dynamic> pointCloudMx = cloud->getMatrixXfMap(4, 8, 0).cast<double>();*/
	//// Print the mapped matrix
	//std::cout << "Point cloud matrix:" << std::endl;
	//std::cout << normalMx << std::endl;

	//for (size_t i = 0; i < normalMx.cols(); ++i)
	//{
	//	if (!normalMx.col(i)[2])
	//	{
	//		Eigen::Vector2d point;
	//		point.x() = normalMx.col(i)[0];
	//		point.y() = normalMx.col(i)[1];

	//		std::cout << point << std::endl;
	//	}
	//}
	//Eigen::MatrixXf resMx = eigen_map.topRows(2);

	//resMx.row(0) = (resMx.row(0).array() + 1) * 0.5 * 20;
	//resMx.row(1) = 10 - 1 - (resMx.row(1).array() + 1) * 0.5 * 10;

	//std::cout << "transfer cloud matrix:" << std::endl;
	//std::cout << resMx << std::endl;

	//// 转为std::vector<Eigen::Vector2d>
	//std::vector<Eigen::Vector2d> vec2DPoints;
	//vec2DPoints.reserve(resMx.cols());

	//for (size_t i = 0; i < resMx.cols(); ++i)
	//{
	//	vec2DPoints.emplace_back(resMx.col(i).cast<double>());
	//}


	// Finish
	return (0);
}