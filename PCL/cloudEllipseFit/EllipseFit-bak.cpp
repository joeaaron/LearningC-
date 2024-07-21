#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_ellipse3d.h>// 拟合3D椭圆
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

int main()
{
	// -------------------------加载点云------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("ellipse.pcd", *cloud) < 0)
	{
		PCL_ERROR("Couldn't read file \n");
		return -1;
	}
	// ------------------------RANSAC框架-----------------------------   
	pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>::Ptr Ellipse3D(new pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(Ellipse3D);
	ransac.setDistanceThreshold(0.19);	        // 距离阈值，与模型距离小于0.01的点作为内点
	ransac.setMaxIterations(100);		        // 最大迭代次数
	ransac.computeModel();				        // 拟合3D椭圆
	pcl::IndicesPtr inliers(new vector <int>());// 存储内点索引的向量
	ransac.getInliers(*inliers);			    // 提取内点对应的索引
	// -----------------根据索引提取椭圆上的点------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr Ellipse_3D(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *Ellipse_3D);
	Eigen::VectorXf coeff;
	ransac.getModelCoefficients(coeff);
	// -------------------输出空间椭圆的参数--------------------------
	cout << "椭圆中心的X坐标：" << coeff[0] << "\n"
		<< "椭圆中心的Y坐标：" << coeff[1] << "\n"
		<< "椭圆中心的Z坐标：" << coeff[2] << "\n"
		<< "沿椭圆局部u轴的半长轴长度：" << coeff[3] << "\n"
		<< "沿椭圆局部v轴的半短轴长度：" << coeff[4] << "\n"
		<< "法线方向的X坐标:" << coeff[5] << "\n"
		<< "法线方向的Y坐标:" << coeff[6] << "\n"
		<< "法线方向的Z坐标:" << coeff[7] << "\n"
		<< "椭圆局部u轴的X坐标:" << coeff[8] << "\n"
		<< "椭圆局部u轴的Y坐标:" << coeff[9] << "\n"
		<< "椭圆局部u轴的Z坐标:" << coeff[10] << "\n"
		<< endl;
	// ---------------如果内点不存在则不进行可视化--------------------
	if (Ellipse_3D->size() == 0)
	{
		cerr << "不存在内点!!!" << endl;
	}
	else
	{
		cout << "拟合后的点数：" << Ellipse_3D->size();
		//-------------------------结果可视化--------------------------
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(u8"拟合三维椭圆"));
		viewer->setBackgroundColor(255, 255, 255);
		// 原始点云
		viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
		// 拟合出的点云
		viewer->addPointCloud<pcl::PointXYZ>(Ellipse_3D, "elipse3D");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "elipse3D");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "elipse3D");

		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}

	return 0;
}

