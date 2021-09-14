#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
 
#include <iostream>
#include <vector>
#include <ctime>
 
int main(int argc, char** argv)
{
	srand(time(NULL)); //��ϵͳʱ���ʼ���������
 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 
	// Generate pointcloud data �����������
	cloud->width = 1000; //��������
	cloud->height = 1; //����Ϊ�������
	cloud->points.resize(cloud->width * cloud->height);
 
	for (size_t i = 0; i < cloud->points.size(); ++i) //ѭ������������
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}
 
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; //����kd-tree
 
	kdtree.setInputCloud(cloud);			//���������ռ�
 
	pcl::PointXYZ searchPoint;		//�����ѯ�㲢�����ֵ
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
 
	// K nearest neighbor search k ��������
	int K = 10;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	std::cout << "K nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with K=" << K << std::endl;
 
	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) //ִ��k��������
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			<< " " << cloud->points[pointIdxNKNSearch[i]].y
			<< " " << cloud->points[pointIdxNKNSearch[i]].z
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}
 
	// Neighbors within radius search �뾶r�ڽ���������ʽ
 
	std::vector<int> pointIdxRadiusSearch; //�洢��������
	std::vector<float> pointRadiusSquaredDistance; //�洢���ڶ�Ӧ�ľ���ƽ��
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