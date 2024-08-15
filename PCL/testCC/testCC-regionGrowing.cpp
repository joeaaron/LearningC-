#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <numeric>
#include <Eigen/Dense>

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
	for (const auto& pt :points)
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
bool IsRightSide(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& candidate) {
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

	for (int i = 1; i < n; ++i) {
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
			for (int index : candidateIndices) {
				if (IsRightSide(centroid, currentPoint, points[index])) {
					chosenIndex = index;
					break;
				}
			}

			if (chosenIndex != -1) {
				used[chosenIndex] = true;
				currentPoint = points[chosenIndex];
				result.push_back(currentPoint);
			}
		}
	}


	return result;
}

// ������������ȷ����ʱ�뷽��
std::vector<Eigen::Vector2d> RegionGrowthSort(const std::vector<Eigen::Vector2d>& points)
{
	int n = points.size();
	if (n == 0) return {};

	Eigen::Vector2d centroid = CalculateCentroid(points);

	// ����ÿ����ļ���
	std::vector<Eigen::Vector2d> sorted_points = points;
	std::vector<int> indices(n);
	std::iota(indices.begin(), indices.end(), 0);

	// �Ӽ�����С�ĵ㿪ʼ
	std::sort(indices.begin(), indices.end(), [&](int i1, int i2) {
		return ComputeAngle(points[i1], centroid) < ComputeAngle(points[i2], centroid);
		});

	std::vector<bool> used(n, false);
	std::vector<Eigen::Vector2d> result;

	int currentIndex = indices[0];
	used[currentIndex] = true;
	result.push_back(points[currentIndex]);

	for (int i = 1; i < n; ++i) {
		double minDist = std::numeric_limits<double>::infinity();
		int nextIndex = -1;

		for (int j = 0; j < n; ++j)
		{
			if (!used[j] && Distance(points[currentIndex], points[j]) < minDist) 
			{
				// ȷ����ʱ�뷽��
				if (result.size() > 1 && !IsCounterClockwise(result[result.size() - 2], points[currentIndex], points[j])) {
					continue;
				}
				minDist = Distance(points[currentIndex], points[j]);
				nextIndex = j;
			}
		}

		if (nextIndex != -1) {
			used[nextIndex] = true;
			result.push_back(points[nextIndex]);
			currentIndex = nextIndex;
		}
	}

	// �պ϶���Σ���ѡ��
	result.push_back(result[0]);

	return result;
}

int main() 
{	
	std::vector<Eigen::Vector2d> points;
	// ʾ���㼯-1
	//points =
	//{ 
	//	{1, 1}, {2, 2}, {2, 1}, {1.1, 1.1}, {1, 2} , {1.5, 1.2}
	//};

	// ʾ���㼯-2
	points =
	{
		{1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {2.5, 2},
		{5, 1}, {5, 2}, {4, 2}, {3, 2}, {2, 2}, {2, 2.5},
		{2, 3}, {1, 3}, {1, 2}, {1, 1}, {1.1, 0},
	};

	auto sortedPoints = RegionGrowthRightSide(points);

	// ��������ĵ�
	for (const auto& point : sortedPoints) {
		std::cout << "(" << point[0] << ", " << point[1] << ")\n";
	}

	return 0;
}