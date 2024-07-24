#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <pcl/io/pcd_io.h>

using namespace Eigen;
using namespace cv;
using namespace std;

// 将三维点集降维到二维
std::vector<cv::Point2f> proj2D(const std::vector<Eigen::Vector3d>& points3D)
{
	Eigen::MatrixXd data(points3D.size(), 3); 
	for (size_t i = 0; i < points3D.size(); ++i)
	{
		data(i, 0) = points3D[i].x();
		data(i, 1) = points3D[i].y(); 
		data(i, 2) = points3D[i].z();

		//PCA分析
		Eigen::Vector3d mean = data.colwise().mean();
		Eigen::MatrixXd centered = data.rowwise() - mean.transpose() ;
		Eigen::MatrixXd cov = centered.adjoint() * centered;
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);

		//投影到主成分形成的平面上
		std::vector<cv::Point2f> points2D; 
		for (size_t i = 0; i < points3D.size(); ++i)
		{
			Eigen::Vector3d point = data.row(i).transpose();
			points2D.emplace_back(static_cast<float>(point.dot(eig.eigenvectors().col(2))),
				static_cast<float>(point.dot(eig.eigenvectors().col(1))));	
		}

		return points2D;
	}
}

vector<cv::Point2f> Proj2D(const vector<Eigen::Vector3d>& points3D) 
{
	int num_points = points3D.size();

	// Convert points3D vector to MatrixXd
	MatrixXd points(3, num_points);
	for (int i = 0; i < num_points; ++i) {
		points.col(i) = points3D[i];
	}

	// Compute mean vector
	Vector3d mean = points.rowwise().mean();

	// Subtract mean from each point
	MatrixXd centered = points.colwise() - mean;

	// Compute covariance matrix
	Matrix3d cov = centered * centered.transpose() / num_points;

	// Perform eigen decomposition of covariance matrix
	SelfAdjointEigenSolver<Matrix3d> eigensolver(cov);
	if (eigensolver.info() != Success) {
		cerr << "Failed to compute eigen decomposition!" << endl;
		return vector<Point2f>();
	}

	// Retrieve eigenvectors corresponding to largest eigenvalues (first two)
	Matrix3d eigenvectors = eigensolver.eigenvectors();

	// Project each centered point onto the plane formed by the first two eigenvectors
	vector<Point2f> projected_points(num_points);
	for (int i = 0; i < num_points; ++i) {
		Vector3d point = centered.col(i);
		Vector2d projection = eigenvectors.block<3, 2>(0, 0).transpose() * point;
		projected_points[i] = Point2f(static_cast<float>(projection(0)), static_cast<float>(projection(1)));
	}

	return projected_points;
}

// 使用openCV拟合二维椭圆
cv::RotatedRect FitEllipseToProjectedPoints(const std::vector<cv::Point2f>& points2D)
{
	if (points2D.size() < 5) 
	{
		std::cout << "需要至少5个点来拟合椭圆";
	}

	return cv::fitEllipse(points2D);
}

std::vector<Eigen::Vector3d> PointCloud2Vector3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::vector<Eigen::Vector3d> vectors(cloud->points.size());

	// 使用 OpenMP 并行化循环
#pragma omp parallel for
	for (int i = 0; i < cloud->points.size(); ++i)
	{
		vectors[i] = Eigen::Vector3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
	}

	return vectors;

}

int main()
{
	// -------------------------加载点云------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("ellipse.pcd", *cloud) < 0)
	{
		PCL_ERROR("Couldn't read file \n");
		return -1;
	}

	std::vector<Eigen::Vector3d> points = PointCloud2Vector3d(cloud);
	// 将三维点降维到二维
	std::vector<cv::Point2f> points2D = Proj2D(points);

	//在二维平面上拟合椭圆
	try
	{
		cv::RotatedRect ellipse = FitEllipseToProjectedPoints(points2D);
		std::cout << "椭圆中心:(" << ellipse.center.x << "," << ellipse.center.y << ")" << std::endl;
		std::cout << "椭圆宽高:(" << ellipse.size.width << "," << ellipse.size.height << ")" << std::endl;
		std::cout << "椭圆旋转角度:(" << ellipse.angle << ")" << std::endl;
	}

	catch (const std::runtime_error& e)
	{
		std::cerr << "错误:" << e.what()<< std::endl;
	}

	return 0;
}