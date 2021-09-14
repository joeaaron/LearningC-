#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
 
#include <iostream>
#include <vector>
#include <ctime>
 
int main(int argc, char** argv)
{
	srand(time(NULL)); //用系统时间初始化随机种子
 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 
	// Generate pointcloud data 随机点云生成
	cloud->width = 1000; //点云数量
	cloud->height = 1; //点云为无序点云
	cloud->points.resize(cloud->width * cloud->height);
 
	for (size_t i = 0; i < cloud->points.size(); ++i) //循环填充点云数据
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}
 
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; //创建kd-tree
 
	kdtree.setInputCloud(cloud);			//设置搜索空间
 
	pcl::PointXYZ searchPoint;		//定义查询点并赋随机值
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
 
	// K nearest neighbor search k 近邻搜索
	int K = 10;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	std::cout << "K nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with K=" << K << std::endl;
 
	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) //执行k近邻搜索
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			<< " " << cloud->points[pointIdxNKNSearch[i]].y
			<< " " << cloud->points[pointIdxNKNSearch[i]].z
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}
 
	// Neighbors within radius search 半径r内近邻搜索方式
 
	std::vector<int> pointIdxRadiusSearch; //存储近邻索引
	std::vector<float> pointRadiusSquaredDistance; //存储近邻对应的距离平方
	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
 
	std::cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << std::endl;
 
 
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}
	return 0;
}