#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <pcl/console/time.h> 
using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

void IsPointSetSimilar(std::vector<bool>& label, const PointCloud::Ptr& cloud, const PointCloud::Ptr& target, double threshold)
{
	pcl::KdTreeFLANN<Point> kdtree;
	kdtree.setInputCloud(cloud);

	for (int i = 0; i < target->points.size(); ++i)
	{
		if (label[i])
		{
			continue;
		}
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);

		if (kdtree.radiusSearch(target->points[i], threshold, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			label[i] = true;
		}
		else 
		{
			label[i] = false;
		}
	}
}

void AddPointSet(std::vector<PointCloud::Ptr>& existing_sets, const PointCloud::Ptr& new_set, double threshold) 
{
	std::vector<bool> label(new_set->points.size(), false);
	for (int i = 0; i < existing_sets.size(); ++i) 
	{
		IsPointSetSimilar(label, existing_sets[i], new_set, threshold);
	}

	PointCloud::Ptr temSet(new PointCloud);
	for (int i = 0; i < label.size(); ++i)
	{
		if (!label[i])
		{
			temSet->points.emplace_back(new_set->points[i]);
		}
	}
	existing_sets.push_back(temSet);
}

int main()
{
	std::vector<PointCloud::Ptr> existing_sets;

	PointCloud::Ptr cloud1(new PointCloud);
	cloud1->points.push_back(Point(0, 0, 0));
	cloud1->points.push_back(Point(1, 1, 1));
	cloud1->points.push_back(Point(2, 2, 2));
	existing_sets.push_back(cloud1);

	PointCloud::Ptr cloud2(new PointCloud);
	cloud2->points.push_back(Point(10, 10, 10));
	cloud2->points.push_back(Point(11, 11, 11));
	cloud2->points.push_back(Point(12, 12, 12));
	existing_sets.push_back(cloud2);

	pcl::console::TicToc time;
	time.tic();

	PointCloud::Ptr new_set(new PointCloud);
	new_set->points.push_back(Point(1.500001, 1.50001, 1.50001));
	new_set->points.push_back(Point(1.000001, 1.000001, 1.000001));
	new_set->points.push_back(Point(12, 12, 12));

	AddPointSet(existing_sets, new_set, 1e-5);

	new_set.reset(new PointCloud);
	new_set->points.push_back(Point(20, 20, 20));
	new_set->points.push_back(Point(1.500001, 1.50001, 1.50001));
	new_set->points.push_back(Point(22, 22, 22));

	AddPointSet(existing_sets, new_set, 1e-5);

	std::cout << "去重重复点算法用时： " << time.toc() << " ms" << std::endl;

	for (const auto& set : existing_sets) {
		for (const auto& point : set->points) {
			std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ") ";
		}
		std::cout << std::endl;
	}

	return 0;
}