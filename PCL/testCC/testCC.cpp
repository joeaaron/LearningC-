// 标准文件
#include <string>
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <chrono>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

//若点a大于点b,即点a在点b顺时针方向,返回true,否则返回false
bool PointCmp(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& center)
{
	if (a.x() >= 0 && b.x() < 0)
		return true;
	if (a.x() == 0 && b.x() == 0)
		return a.y() > b.y();
	//向量OA和向量OB的叉积
	int det = (a.x() - center.x()) * (b.y() - center.y()) - (b.x() - center.x()) * (a.y() - center.y());
	if (det < 0)
		return true;
	if (det > 0)
		return false;
	//向量OA和向量OB共线，以距离判断大小
	int d1 = (a.x() - center.x()) * (a.x() - center.x()) + (a.y() - center.y()) * (a.y() - center.y());
	int d2 = (b.x() - center.x()) * (b.x() - center.y()) + (b.y() - center.y()) * (b.y() - center.y());
	return d1 > d2;
}

void ClockwiseSortPoints(std::vector<Eigen::Vector2d>& vPoints)
{
	//计算重心
	Eigen::Vector2d center;
	double x = 0, y = 0;
	for (int i = 0; i < vPoints.size(); i++)
	{
		x += vPoints[i].x();
		y += vPoints[i].y();
	}
	center.x() = x / vPoints.size();
	center.y() = y / vPoints.size();

	//冒泡排序
	for (int i = 0; i < vPoints.size() - 1; i++)
	{
		for (int j = 0; j < vPoints.size() - i - 1; j++)
		{
			if (PointCmp(vPoints[j], vPoints[j + 1], center))
			{
				Eigen::Vector2d tmp = vPoints[j];
				vPoints[j] = vPoints[j + 1];
				vPoints[j + 1] = tmp;
			}
		}
	}
}

// 计算两点间的欧几里得距离
double Distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
{
	return (p1 - p2).norm();
}

// 计算点相对于质心的极角
double ComputeAngle(const Eigen::Vector2d& point, const Eigen::Vector2d& centroid)
{
	return std::atan2(point[1] - centroid[1], point[0] - centroid[0]);
}

// 计算质心
Eigen::Vector2d CalculateCentroid(const std::vector<Eigen::Vector2d>& points)
{
	Eigen::Vector2d centroid(0, 0);
	for (const auto& pt : points)
	{
		centroid += pt;
	}
	return centroid / points.size();
}

// 确定是否在逆时针方向
bool IsCounterClockwise(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3)
{
	return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0]) > 0;
}

// 计算向量叉积判断点是否在右侧
bool IsRightSide(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& candidate) 
{
	// 使用叉积来判断 candidate 是否在 (p1 -> p2) 的右侧
	return (p2 - p1).x() * (candidate - p1).y() - (p2 - p1).y() * (candidate - p1).x() > 0;
}

// 找到最左下角的点
int FindBottomLeft(const std::vector<Eigen::Vector2d>& points)
{
	int bottomLeftIndex = 0;
	for (int i = 1; i < points.size(); ++i) {
		if (points[i].y() < points[bottomLeftIndex].y() ||
			(points[i].y() == points[bottomLeftIndex].y() && points[i].x() < points[bottomLeftIndex].x())) {
			bottomLeftIndex = i;
		}
	}
	return bottomLeftIndex;
}

void IsDuplicatedPoints(std::vector<bool>& vbFlags,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudNew, double dDistThresh)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (int i = 0; i < cloudNew->size(); ++i)
	{
		if (vbFlags[i])
		{
			continue;
		}
		std::vector<int> vIndices;					// 存储搜索到的点的索引
		std::vector<float> vdDists;					// 存储对应点的距离的平方

		if (tree.radiusSearch(cloudNew->points[i], dDistThresh, vIndices, vdDists) > 0)
		{
			vbFlags[i] = true;
		}
		else
		{
			vbFlags[i] = false;
		}
	}
}

bool RemoveDuplicatedForPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cloudInOut,
	std::vector<int>& vIdxOut,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudNew,
	double dDistThresh)
{
	try
	{
		std::vector<bool> vbFlags(cloudNew->size(), false);
		for (int i = 0; i < cloudInOut.size(); ++i)
		{
			if (cloudInOut[i]->points.empty())
			{
				break;
			}

			IsDuplicatedPoints(vbFlags, cloudInOut[i], cloudNew, dDistThresh);
		}

		std::vector<int> localIndices;
		localIndices.reserve(vbFlags.size());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAdded(new pcl::PointCloud<pcl::PointXYZ>);

		for (int i = 0; i < vbFlags.size(); ++i)
		{
			if (!vbFlags[i])
			{
				localIndices.emplace_back(i);
				cloudAdded->points.emplace_back(cloudNew->points[i]);
			}

		}

		localIndices.shrink_to_fit();
		vIdxOut.swap(localIndices);

		cloudInOut.emplace_back(cloudAdded);
	}
	catch (...)
	{
		return false;
	}

	return !vIdxOut.empty();
}

// 区域生长排序，确保点在右侧
std::vector<Eigen::Vector2d> RegionGrowthRightSide(const std::vector<Eigen::Vector2d>& points)
{
	int n = points.size();
	if (n == 0) return {};

	std::vector<bool> used(n, false);
	std::vector<Eigen::Vector2d> result;

	// 计算点集的中心点
	Eigen::Vector2d centroid = CalculateCentroid(points);

	// 找到最左下角的点作为起始点
	int startIndex = 0;
	for (int i = 1; i < n; ++i) {
		if (points[i].y() < points[startIndex].y() ||
			(points[i].y() == points[startIndex].y() && points[i].x() < points[startIndex].x())) {
			startIndex = i;
		}
	}

	Eigen::Vector2d currentPoint = points[startIndex];
	used[startIndex] = true;
	result.push_back(currentPoint);

	for (int k = 1; k < n; ++k) {
		double minDist = std::numeric_limits<double>::infinity();
		std::vector<int> candidateIndices;

		// 找最近点
		for (int j = 0; j < n; ++j) {
			if (!used[j]) {
				double dist = Distance(currentPoint, points[j]);
				if (dist < minDist) {
					minDist = dist;
					candidateIndices.clear();
					candidateIndices.push_back(j);
				}
				else if (dist == minDist) {
					candidateIndices.push_back(j);
				}
			}
		}

		// 如果只有一个最近点，直接选中
		if (candidateIndices.size() == 1) {
			int nextIndex = candidateIndices[0];
			used[nextIndex] = true;
			currentPoint = points[nextIndex];
			result.push_back(currentPoint);
		}
		else {
			// 如果有多个最近点，根据方向选择合适的点
			int chosenIndex = -1;
			for (int index : candidateIndices) 
			{
				if (IsRightSide(centroid, currentPoint, points[index])) 
				{
					chosenIndex = index;
					break;
				}
			}

			if (chosenIndex != -1) 
			{
				used[chosenIndex] = true;
				currentPoint = points[chosenIndex];
				result.push_back(currentPoint);
			}
		}
	}
	return result;
}


// 计算向量叉积，判断方向
bool IsRightSide(const Eigen::Vector3d& currentPoint, 
	const Eigen::Vector3d& referencePoint, 
	const Eigen::Vector3d& candidate) 
{
	Eigen::Vector3d v1 = currentPoint - referencePoint;
	Eigen::Vector3d v2 = candidate - referencePoint;
	Eigen::Vector3d crossProduct = v1.cross(v2);
	return crossProduct.z() < 0;  // 判断在Z轴上的方向（假设排序方向是逆时针）
}

// 使用 k-d Tree 查找最近点并进行排序
PointCloud::Ptr SortPointsUsingKDTree(const PointCloud::Ptr& cloud)
{
	// 构建 k-d Tree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	// 计算点云密度
	double dAvgDis = 0.0;
	int nNum = 0;
	for (int i = 0; i < cloud->size(); ++i)
	{
		std::vector<int> indiceId;
		std::vector<float> disSquare;

		if (kdtree.nearestKSearch(cloud->points[i], 2, indiceId, disSquare) > 0)
		{
			dAvgDis += sqrt(disSquare[1]);
			nNum++;
		}
	}
	dAvgDis /= nNum;


	// 结果点集
	PointCloud::Ptr sortedPoints(new PointCloud);
	
	// 选择起始点，例如选择最左下角的点
	pcl::PointXYZ startPoint = cloud->points[0];  // 假设选择第一个点作为起始点
	sortedPoints->points.push_back(startPoint);

	std::vector<bool> used(cloud->points.size(), false);
	used[0] = true;

	pcl::PointXYZ currentPoint = startPoint;

	for (size_t i = 1; i < cloud->points.size(); ++i) 
	{
		float searchRadius = dAvgDis;						//0.1f：搜索半径，可根据点云密度调整
		bool foundValidPoint = false;

		while (!foundValidPoint)
		{
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			// kd-tree 半径搜索，控制搜索范围
			if (kdtree.radiusSearch(currentPoint, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= 0)
			{
				searchRadius *= 2;
			} 
			else
			{
				int nearestIdx = -1;
				float minDistance = std::numeric_limits<float>::max();

				for (int j = 0; j < pointIdxRadiusSearch.size(); ++j)
				{
					int idx = pointIdxRadiusSearch[j];
					if (!used[idx] && pointRadiusSquaredDistance[j] < minDistance)
					{
						nearestIdx = idx;
						minDistance = pointRadiusSquaredDistance[j];
					}
				}
				// 如果找到合适的最近点
				if (nearestIdx != -1)
				{
					pcl::PointXYZ nearestPoint = cloud->points[nearestIdx];
					sortedPoints->points.push_back(nearestPoint);
					used[nearestIdx] = true;
					currentPoint = nearestPoint;
					foundValidPoint = true;
				}
				else
				{
					searchRadius *= 2;		// 动态调整半径
				}
			}
		}
	}

	return sortedPoints;
}


int main() 
{
	// ****************************获取数据******************************
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PolygonMesh mesh;
	std::string fnameS = R"(Test0819.pcd)";   //Test0815-rd.pcd

	//支持pcd与ply两种格式
	if (fnameS.substr(fnameS.find_last_of('.') + 1) == "pcd") {
		pcl::io::loadPCDFile(fnameS, *pc);
	}
	else if (fnameS.substr(fnameS.find_last_of('.') + 1) == "ply") {
		pcl::io::loadPLYFile(fnameS, *pc);
	}
	else if (fnameS.substr(fnameS.find_last_of('.') + 1) == "stl") {
		pcl::io::loadPolygonFileSTL(fnameS, mesh);
	}

	cout << "Points num = " << pc->points.size() << std::endl;

	// 创建KD-Tree对象用于搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(pc);

	// 计算点云密度
	double dAvgDis = 0.0;
	int nNum = 0;
	for (int i = 0; i < pc->size(); ++i)
	{
		std::vector<int> indiceId;
		std::vector<float> disSquare;

		if (tree->nearestKSearch(pc->points[i], 2, indiceId, disSquare) > 0)
		{
			dAvgDis += sqrt(disSquare[1]);
			nNum++;
		}
	}
	dAvgDis /= nNum;

	// 欧几里德聚类提取器
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(3);				// 设置近邻搜索的搜索半径
	ec.setMinClusterSize(200);				// 设置一个聚类需要的最少点数目
	ec.setMaxClusterSize(100000);			// 设置一个聚类需要的最大点数目
	ec.setSearchMethod(tree);
	ec.setInputCloud(pc);
	ec.extract(cluster_indices);

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vCloud;
	for (const auto& indices : cluster_indices)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (const auto& index : indices.indices)
			cloud_cluster->points.push_back(pc->points[index]);

		// 进行点排序
		auto startOp = std::chrono::high_resolution_clock::now();
		PointCloud::Ptr sortedPoints = SortPointsUsingKDTree(cloud_cluster);
		auto endOp = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsedOp = endOp - startOp;
		std::cout << "区域生长点排序算法用时: " << elapsedOp.count() << " seconds" << std::endl;

		// 创建可视化器
		pcl::visualization::PCLVisualizer viewer("Line Viewer");

		// 遍历点云并顺序连接相邻的点
		for (size_t i = 0; i < sortedPoints->points.size() - 1; ++i) 
		{
			std::string line_id = "line_" + std::to_string(i);
			double dis = pcl::euclideanDistance(sortedPoints->points[i], sortedPoints->points[i + 1]);
			if ( dis > 10 * dAvgDis)
				continue;

			viewer.addLine(sortedPoints->points[i], sortedPoints->points[i + 1], line_id);
		}
		// 连接最后一个点到第一个点，形成闭环
		//viewer.addLine(sortedPoints->points.back(), sortedPoints->points.front(), "line_close");
		viewer.resetCamera();

		// 运行可视化器
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
		}

		vCloud.emplace_back(cloud_cluster);
	}

	return 0;
}

