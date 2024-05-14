#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/boundary.h>
#include <math.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/boundary.h>
#include <pcl/io/ply_io.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/console/time.h> 
#include <pcl/surface/concave_hull.h>  

#define ENABLE_DISPLAY 0		// 定义一个宏，用于控制显示状态

void CloudBoundaryHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::console::TicToc time;
	time.tic();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud); // 输入点云为投影后的点云
	chull.setAlpha(1.2);        // 设置alpha值为0.1
	chull.reconstruct(*cloud_hull);

	cout << "AS提取边界点个数为: " << cloud_hull->points.size() << endl;
	cout << "AS提取边界点用时： " << time.toc() / 1000 << " 秒" << endl;
	pcl::PCDWriter writer;
	writer.write("hull.pcd", *cloud_hull, false);

#if ENABLE_DISPLAY
	//-----------------结果显示---------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud boundary extraction - AS"));

	int v1(0), v2(0);
	viewer->setWindowName("alpha_shapes提取边界");
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	viewer->addPointCloud(cloud_hull, "cloud_boundary");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_boundary");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_boundary");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(1000);
	}

#endif
}

int CloudBoundaryAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::console::TicToc time;
	time.tic();

	// 1 计算法向量
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr  normals(new  pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setSearchMethod(tree);
	normalEstimation.setRadiusSearch(0.02);  // 法向量的半径
	normalEstimation.compute(*normals);

	/*pcl计算边界*/
	pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>); //声明一个boundary类指针，作为返回值
	boundaries->resize(cloud->size()); //初始化大小
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimation; //声明一个BoundaryEstimation类
	boundary_estimation.setInputCloud(cloud); //设置输入点云
	boundary_estimation.setInputNormals(normals); //设置输入法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ptr(new pcl::search::KdTree<pcl::PointXYZ>);
	boundary_estimation.setSearchMethod(kdtree_ptr); //设置搜寻k近邻的方式
	boundary_estimation.setKSearch(30); //设置k近邻数量
	boundary_estimation.setAngleThreshold(M_PI * 0.6); //设置角度阈值，大于阈值为边界
	boundary_estimation.compute(*boundaries); //计算点云边界，结果保存在boundaries中

	cout << "AC提取边界点个数为   ：  " << boundaries->size() << endl;
	cout << "AC提取边界点用时： " << time.toc() / 1000 << " 秒" << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_visual->resize(cloud->size());
	for (size_t i = 0; i < cloud->size(); i++)
	{
		cloud_visual->points[i].x = cloud->points[i].x;
		cloud_visual->points[i].y = cloud->points[i].y;
		cloud_visual->points[i].z = cloud->points[i].z;
		if (boundaries->points[i].boundary_point != 0)
		{
			cloud_visual->points[i].r = 255;
			cloud_visual->points[i].g = 0;
			cloud_visual->points[i].b = 0;
			cloud_boundary->push_back(cloud_visual->points[i]);
		}
		else
		{
			cloud_visual->points[i].r = 255;
			cloud_visual->points[i].g = 255;
			cloud_visual->points[i].b = 255;
		}
	}
#if ENABLE_DISPLAY
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Cloud boundary extraction - AC"));

	int v1(0);
	MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
	MView->addText("Raw point clouds", 10, 10, "v1_text", v1);
	int v2(0);
	MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
	MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
	MView->addText("Boudary point clouds", 10, 10, "v2_text", v2);

	MView->addPointCloud<pcl::PointXYZRGB>(cloud_visual, "sample cloud", v1);
	MView->addPointCloud<pcl::PointXYZRGB>(cloud_boundary, "cloud_boundary", v2);

	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_boundary", v2);

	MView->addCoordinateSystem(1.0);
	MView->initCameraParameters();

	MView->spin();
#endif
	/*pcl::io::savePCDFileBinaryCompressed("all.pcd", *cloud_visual);
	pcl::io::savePCDFileBinaryCompressed("pcd", *cloud_boundary);*/
	return 0;
}

int estimateBorders(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float re, float reforn)
{

	pcl::PointCloud<pcl::Boundary> boundaries;
	//使用角度标准确定一组点是否位于边界上。该代码利用输入数据集中每个点估计的表面法线
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
	normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud));
	normEst.setRadiusSearch(reforn);
	normEst.compute(*normals);

	boundEst.setInputCloud(cloud);
	boundEst.setInputNormals(normals);
	boundEst.setRadiusSearch(re);
	boundEst.setAngleThreshold(M_PI / 4);
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	boundEst.compute(boundaries);

	for (int i = 0; i < cloud->points.size(); i++)
	{

		if (boundaries[i].boundary_point > 0)
		{
			cloud_boundary->push_back(cloud->points[i]);
		}
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("点云边界提取"));

	int v1(0);
	MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
	MView->addText("Raw point clouds", 10, 10, "v1_text", v1);
	int v2(0);
	MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
	MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
	MView->addText("Boudary point clouds", 10, 10, "v2_text", v2);

	MView->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
	MView->addPointCloud<pcl::PointXYZ>(cloud_boundary, "cloud_boundary", v2);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_boundary", v2);
	MView->addCoordinateSystem(1.0);
	MView->initCameraParameters();

	MView->spin();

	return 0;
}
int
main(int argc, char** argv)
{
	//是以当前时间为种子，产生随意数。
	//其中,time(NULL)用来获取当前时间，本质上得到的是一个大整数，然后用这个数来随机数。
	//https://www.cnblogs.com/hangaozu/p/8280397.html 介绍这个函数
	//srand(time(NULL));

	//float re, reforn;
	//re = std::atof(argv[2]);//把字符串转换成浮点数
	//reforn = std::atof(argv[3]);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::io::loadPCDFile(argv[1], *cloud_src);
	//estimateBorders(cloud_src, re, reforn);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud) == -1)		// sac_plane_test.pcd | Scan_0511_1713.pcd
	{
		PCL_ERROR("点云读取失败 \n");
		return (-1);
	}

	// 均匀下采样
	pcl::UniformSampling<pcl::PointXYZ> us;
	us.setInputCloud(cloud);
	us.setRadiusSearch(0.01f);
	us.filter(*cloudFiltered);

	// Angle Criterion算法
	CloudBoundaryAC(cloudFiltered);
	
	// Alpha Shape算法
	CloudBoundaryHull(cloudFiltered);

	return 0;
}