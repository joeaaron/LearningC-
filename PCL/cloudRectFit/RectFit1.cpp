#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/point_cloud.h>

using namespace std;
using namespace pcl;

// 计算质心
Eigen::Vector4f computeCentroid(const PointCloud<PointXYZ>::Ptr& cloud) {
	Eigen::Vector4f centroid;
	compute3DCentroid(*cloud, centroid);
	return centroid;
}

// 计算协方差矩阵
Eigen::Matrix3f computeCovarianceMatrix(const PointCloud<PointXYZ>::Ptr& cloud, const Eigen::Vector4f& centroid) {
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
	return covariance;
}

// 分割点集为四条边的点集
void SplitEdges1(const PointCloud<PointXYZ>::Ptr& cloud, const Eigen::Vector4f& centroid, const Eigen::Vector3f& longDir, const Eigen::Vector3f& shortDir,
	PointCloud<PointXYZ>::Ptr& edge1, PointCloud<PointXYZ>::Ptr& edge2, PointCloud<PointXYZ>::Ptr& edge3, PointCloud<PointXYZ>::Ptr& edge4) {
	// 投影到长边和短边方向
	float minLong = numeric_limits<float>::max(), maxLong = -numeric_limits<float>::max();
	float minShort = numeric_limits<float>::max(), maxShort = -numeric_limits<float>::max();

	for (const auto& point : cloud->points) {
		Eigen::Vector3f centered(point.x - centroid[0], point.y - centroid[1], point.z - centroid[2]);
		float longProj = centered.dot(longDir);
		float shortProj = centered.dot(shortDir);

		if (longProj < minLong) minLong = longProj;
		if (longProj > maxLong) maxLong = longProj;
		if (shortProj < minShort) minShort = shortProj;
		if (shortProj > maxShort) maxShort = shortProj;
	}

	// 分配点到四条边
	for (const auto& point : cloud->points) {
		Eigen::Vector3f centered(point.x - centroid[0], point.y - centroid[1], point.z - centroid[2]);
		float longProj = centered.dot(longDir);
		float shortProj = centered.dot(shortDir);

		if (fabs(longProj - minLong) < fabs(longProj - maxLong)) {
			if (fabs(shortProj - minShort) < fabs(shortProj - maxShort)) {
				edge1->points.push_back(point); // 左下
			}
			else {
				edge2->points.push_back(point); // 左上
			}
		}
		else {
			if (fabs(shortProj - minShort) < fabs(shortProj - maxShort)) {
				edge3->points.push_back(point); // 右下
			}
			else {
				edge4->points.push_back(point); // 右上
			}
		}
	}
}

// 分割点集为左右和上下的点集
void SplitEdges(const PointCloud<PointXYZ>::Ptr & cloud, const Eigen::Vector4f & centroid, const Eigen::Vector3f & longDir, const Eigen::Vector3f & shortDir,
	PointCloud<PointXYZ>::Ptr & left, PointCloud<PointXYZ>::Ptr & right, PointCloud<PointXYZ>::Ptr & top, PointCloud<PointXYZ>::Ptr & bottom)
{
	// 投影到长边和短边方向
	for (const auto& point : cloud->points) 
	{
		Eigen::Vector3f centered(point.x - centroid[0], point.y - centroid[1], point.z - centroid[2]);
		float longProj = centered.dot(longDir);
		float shortProj = centered.dot(shortDir);

		if (longProj < 0) {
			left->points.push_back(point);
		}
		else {
			right->points.push_back(point);
		}

		if (shortProj < 0) {
			bottom->points.push_back(point);
		}
		else {
			top->points.push_back(point);
		}
	}

	cout << "Left size: " << left->points.size() << endl;
	cout << "Right size: " << right->points.size() << endl;
	cout << "Top size: " << top->points.size() << endl;
	cout << "Bottom size: " << bottom->points.size() << endl;
}

int main() {
	// 创建点云并加载数据
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	if (io::loadPCDFile<PointXYZ>("rect1.pcd", *cloud) == -1) {
		PCL_ERROR("Couldn't read file input.pcd \n");
		return (-1);
	}

	// 计算质心
	Eigen::Vector4f centroid = computeCentroid(cloud);

	// 计算协方差矩阵
	Eigen::Matrix3f covMatrix = computeCovarianceMatrix(cloud, centroid);

	// 主成分分析
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covMatrix);
	Eigen::Matrix3f eigVecs = eigensolver.eigenvectors();
	Eigen::Vector3f longDir = eigVecs.col(2);
	Eigen::Vector3f shortDir = eigVecs.col(1);

	// 分割点集为四条边的点集
	PointCloud<PointXYZ>::Ptr edge1(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr edge2(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr edge3(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr edge4(new PointCloud<PointXYZ>);

	SplitEdges(cloud, centroid, longDir, shortDir, edge1, edge2, edge3, edge4);
	edge1->width = edge1->points.size();
	edge1->height = 1;
	edge1->is_dense = true;

	edge2->width = edge2->points.size();
	edge2->height = 1;
	edge2->is_dense = true;

	edge3->width = edge3->points.size();
	edge3->height = 1;
	edge3->is_dense = true;

	edge4->width = edge4->points.size();
	edge4->height = 1;
	edge4->is_dense = true;
	// 保存结果
	io::savePCDFile("edge1.pcd", *edge1);
	io::savePCDFile("edge2.pcd", *edge2);
	io::savePCDFile("edge3.pcd", *edge3);
	io::savePCDFile("edge4.pcd", *edge4);

	return 0;
}
