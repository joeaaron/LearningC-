#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class PointSetManager 
{
public:
	PointSetManager(double threshold) : distance_threshold(threshold) 
	{
		//kdtree.setInputCloud(existing_points);
	}

	void AddPointSet(const PointCloudT::Ptr& new_points) 
	{
		for (const auto& point : new_points->points) 
		{
			std::vector<int> pointIdxNKNSearch(1);
			std::vector<float> pointNKNSquaredDistance(1);

			// ������ھ���С����ֵ�ĵ㣬����Ӹõ�
			if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) 
			{
				if (pointNKNSquaredDistance[0] < distance_threshold * distance_threshold) {
					continue;
				}
			}

			existing_points->points.push_back(point);
		}
		// ����KD-Tree
		kdtree.setInputCloud(existing_points);
	}

	PointCloudT::Ptr GetExistingPoints() const 
	{
		return existing_points;
	}

private:
	double distance_threshold;
	PointCloudT::Ptr existing_points{ new PointCloudT };
	pcl::KdTreeFLANN<PointT> kdtree;
};