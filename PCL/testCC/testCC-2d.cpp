// ��׼�ļ�
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

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

//����a���ڵ�b,����a�ڵ�b˳ʱ�뷽��,����true,���򷵻�false
bool PointCmp(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& center)
{
	if (a.x() >= 0 && b.x() < 0)
		return true;
	if (a.x() == 0 && b.x() == 0)
		return a.y() > b.y();
	//����OA������OB�Ĳ��
	int det = (a.x() - center.x()) * (b.y() - center.y()) - (b.x() - center.x()) * (a.y() - center.y());
	if (det < 0)
		return true;
	if (det > 0)
		return false;
	//����OA������OB���ߣ��Ծ����жϴ�С
	int d1 = (a.x() - center.x()) * (a.x() - center.x()) + (a.y() - center.y()) * (a.y() - center.y());
	int d2 = (b.x() - center.x()) * (b.x() - center.y()) + (b.y() - center.y()) * (b.y() - center.y());
	return d1 > d2;
}

void ClockwiseSortPoints(std::vector<Eigen::Vector2d>& vPoints)
{
	//��������
	Eigen::Vector2d center;
	double x = 0, y = 0;
	for (int i = 0; i < vPoints.size(); i++)
	{
		x += vPoints[i].x();
		y += vPoints[i].y();
	}
	center.x() = x / vPoints.size();
	center.y() = y / vPoints.size();

	//ð������
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

// ����������ŷ����þ���
double Distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
{
	return (p1 - p2).norm();
}

// �������������ĵļ���
double ComputeAngle(const Eigen::Vector2d& point, const Eigen::Vector2d& centroid)
{
	return std::atan2(point[1] - centroid[1], point[0] - centroid[0]);
}

// ��������
Eigen::Vector2d CalculateCentroid(const std::vector<Eigen::Vector2d>& points)
{
	Eigen::Vector2d centroid(0, 0);
	for (const auto& pt : points)
	{
		centroid += pt;
	}
	return centroid / points.size();
}

// ȷ���Ƿ�����ʱ�뷽��
bool IsCounterClockwise(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3)
{
	return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0]) > 0;
}

// ������������жϵ��Ƿ����Ҳ�
bool IsRightSide(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& candidate) 
{
	// ʹ�ò�����ж� candidate �Ƿ��� (p1 -> p2) ���Ҳ�
	return (p2 - p1).x() * (candidate - p1).y() - (p2 - p1).y() * (candidate - p1).x() > 0;
}

// �ҵ������½ǵĵ�
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
		std::vector<int> vIndices;					// �洢�������ĵ������
		std::vector<float> vdDists;					// �洢��Ӧ��ľ����ƽ��

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

// ������������ȷ�������Ҳ�
std::vector<Eigen::Vector2d> RegionGrowthRightSide(const std::vector<Eigen::Vector2d>& points)
{
	int n = points.size();
	if (n == 0) return {};

	std::vector<bool> used(n, false);
	std::vector<Eigen::Vector2d> result;

	// ����㼯�����ĵ�
	Eigen::Vector2d centroid = CalculateCentroid(points);

	// �ҵ������½ǵĵ���Ϊ��ʼ��
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

		// �������
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

		// ���ֻ��һ������㣬ֱ��ѡ��
		if (candidateIndices.size() == 1) {
			int nextIndex = candidateIndices[0];
			used[nextIndex] = true;
			currentPoint = points[nextIndex];
			result.push_back(currentPoint);
		}
		else {
			// ����ж������㣬���ݷ���ѡ����ʵĵ�
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

int main() 
{
	// ****************************��ȡ����******************************
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PolygonMesh mesh;
	std::string fnameS = R"(Test0815-rd.pcd)";   //pmt.pcd | bunny.pcd | Testcase01.pcd | test.stl | Testcase02.pcd
	//֧��pcd��ply���ָ�ʽ
	if (fnameS.substr(fnameS.find_last_of('.') + 1) == "pcd") {
		pcl::io::loadPCDFile(fnameS, *pc);
	}
	else if (fnameS.substr(fnameS.find_last_of('.') + 1) == "ply") {
		pcl::io::loadPLYFile(fnameS, *pc);
	}
	else if (fnameS.substr(fnameS.find_last_of('.') + 1) == "stl") {
		pcl::io::loadPolygonFileSTL(fnameS, mesh);
	}

	// ����KD-Tree������������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(pc);

	// ŷ����¾�����ȡ��
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(3);				// ���ý��������������뾶
	ec.setMinClusterSize(200);				// ����һ��������Ҫ�����ٵ���Ŀ
	ec.setMaxClusterSize(100000);			// ����һ��������Ҫ��������Ŀ
	ec.setSearchMethod(tree);
	ec.setInputCloud(pc);
	ec.extract(cluster_indices);

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vCloud;
	for (const auto& indices : cluster_indices)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (const auto& index : indices.indices)
			cloud_cluster->points.push_back(pc->points[index]);

		vCloud.emplace_back(cloud_cluster);

		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
	}

	double dZ = pc->points[0].z;
	for (auto& cloud : vCloud)
	{
		// ͶӰ��XOY��
		pcl::ProjectInliers<pcl::PointXYZ> projector;
		projector.setModelType(pcl::SACMODEL_PLANE);
		projector.setInputCloud(cloud);

		// ����ͶӰ�� XOY ƽ���ģ��ϵ��
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		coefficients->values.resize(4);
		coefficients->values[0] = 0; // A (X ����ķ�����)
		coefficients->values[1] = 0; // B (Y ����ķ�����)
		coefficients->values[2] = 1; // C (Z ����ķ�������1 ��ʾ Z �ᴹֱƽ��)
		coefficients->values[3] = 0; // D (ƽ��� D ����)
		projector.setModelCoefficients(coefficients);

		// ִ��ͶӰ
		PointCloud::Ptr cloudProjected(new PointCloud);
		projector.filter(*cloudProjected);

		//����һ��std::vector���ڴ洢 Eigen::Vector2d
		std::vector<Eigen::Vector2d> vPoints2D;
		for (const auto& point : cloudProjected->points)
		{
			Eigen::Vector2d vec(point.x, point.y);
			vPoints2D.push_back(vec);
		}

		auto sortedPoints = RegionGrowthRightSide(vPoints2D);
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr polyLine(new pcl::PointCloud<pcl::PointXYZ>);
		for (int i = 0; i < sortedPoints.size(); ++i)
		{
			polyLine->points.emplace_back(sortedPoints[i][0], sortedPoints[i][1], dZ);
		}

		// �������ӻ���
		pcl::visualization::PCLVisualizer viewer("Line Viewer");
		
		// �������Ʋ�˳���������ڵĵ�
		for (size_t i = 0; i < polyLine->points.size() - 1; ++i) {
			std::string line_id = "line_" + std::to_string(i);
			viewer.addLine(polyLine->points[i], polyLine->points[i + 1], line_id);
		}
		// �������һ���㵽��һ���㣬�γɱջ�
		viewer.addLine(polyLine->points.back(), polyLine->points.front(), "line_close");

		// ���п��ӻ���
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
		}
	}
	
	return 0;
}

