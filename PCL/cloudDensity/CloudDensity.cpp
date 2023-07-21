#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/sample_consensus_prerejective.h>

using namespace std;

double ComputeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	double dResolution = 0.0;
	int nNumOfPoints = 0;
	int nRes = 0;

	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{    //检查是否存在无效点
		if (!pcl::isFinite(cloud->points[i]))
			continue;

		//Considering the second neighbor since the first is the point itself.
		//在同一个点云内进行k近邻搜索时，k=1的点为查询点本身。
		nRes = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nRes == 2)
		{
			dResolution += sqrt(squaredDistances[1]);
			++nNumOfPoints;
		}
	}
	if (nNumOfPoints != 0)
		dResolution /= nNumOfPoints;

	return dResolution;
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("K\\K1At.pcd", *source);

	double dResolution = ComputeCloudResolution(source);

	cout << "点云密度为：" << dResolution << endl;

	return 0;
}

