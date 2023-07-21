#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include "NormalEstimation.h"

using namespace std;

int main(int argc, char** argv)
{
	// -------------------------加载点云-------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("Z:\\CodeProj\\JA\\LearningC++\\resource\\bunny.pcd", *cloud) == -1)
	{
		cerr << "can't read file " << endl;
		return -1;
	}
	// ---------------------计算法向量和曲率---------------------------
	pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
	normals = EstimateNormals(cloud, 0.005, 30); //输入参数为：点云、邻域搜索半径和邻域最大点数
	cout << "点云法向量和曲率计算完毕！！！" << endl;
	pcl::io::savePCDFileBinary("normals.pcd", *normals);

	//---------------------可视化（含法线）-----------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("CloudCompare-XYZNormal viewer"));
	viewer->setWindowName("快速计算点云的法向量");
	viewer->addText("PointNormal", 50, 50, 0, 1, 0, "v1_text");
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal> fildColor(normals, "z");
	viewer->addPointCloud<pcl::PointNormal>(normals, fildColor, "cloud_curvature");
	viewer->addPointCloudNormals<pcl::PointNormal>(normals, 20, 0.02, "normals");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}

