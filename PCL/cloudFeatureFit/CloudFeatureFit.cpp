#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

int
main(int argc, char** argv)
{
	//------------------------------��ȡ��������---------------------------------
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ >);
	pcl::PCDReader reader;
	reader.read("table_scene_mug_stereo_textured.pcd", *cloud);
	cout << "PointCloud has: " << cloud->points.size() << " data points." << endl;

	//--------------------------------ֱͨ�˲�-----------------------------------
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ >);
	pcl::PassThrough<pcl::PointXYZ > pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");//��Z�᲻�ڣ�0��1.5����Χ�ڵĵ���˵�
	pass.setFilterLimits(0, 1.5);
	pass.filter(*cloud_filtered);//ʣ��ĵ㴢����cloud_filtered�к���ʹ��
	cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << endl;
	
	//--------------------------------���㷨��-----------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ >::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ >);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	n.setSearchMethod(tree);
	n.setInputCloud(cloud_filtered);
	n.setKSearch(50);
	n.compute(*normals);
	
	//------------------------------�����ָ����---------------------------------
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
	
	// ------------------------���Ʒָ��ȡƽ���ϵĵ�--------------------------
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.1);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(normals);
	seg.segment(*inliers_plane, *coefficients_plane);//��ȡƽ��ģ��ϵ����ƽ���ϵĵ�
	cout << "Plane coefficients: " << *coefficients_plane << endl;
	
	//----------------------------------��ȡƽ��---------------------------------
	pcl::ExtractIndices<pcl::PointXYZ > extract;
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ >());
	extract.filter(*cloud_plane);
	cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << endl;

	//-------------------------------��ȡԲ����ģ��------------------------------
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ >);
	pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
	
	//��ȡƽ������ĵ�͵�ķ���
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);
	extract_normals.setNegative(true);
	extract_normals.setInputCloud(normals);
	extract_normals.setIndices(inliers_plane);
	extract_normals.filter(*normals2);

	//ΪԲ����ָ���ָ���󣬲����ò���
	seg.setOptimizeCoefficients(true);        //���öԹ��Ƶ�ģ��ϵ����Ҫ�����Ż�
	seg.setModelType(pcl::SACMODEL_CYLINDER); //���÷ָ�ģ��ΪԲ����
	seg.setMethodType(pcl::SAC_RANSAC);       //���ò���RANSAC��Ϊ�㷨�Ĳ������Ʒ���
	seg.setNormalDistanceWeight(0.1);         //���ñ��淨��Ȩ��ϵ��
	seg.setMaxIterations(5000);               //���õ�����������
	seg.setDistanceThreshold(0.05);           //�����ڵ㵽ģ�͵ľ����������ֵ 
	seg.setRadiusLimits(0, 0.1);              //���ù��Ƴ�Բ��ģ�͵İ뾶��Χ
	seg.setInputCloud(cloud_filtered2);
	seg.setInputNormals(normals2);

	//��ȡԲ��ģ��ϵ����Բ���ϵĵ�
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	cout << "Cylinder coefficients: " << *coefficients_cylinder << endl;
	
	//-----------------------------�洢���Ƶ�����ļ�----------------------------
	extract.setInputCloud(cloud_filtered2);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ >);
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty())
		cout << "Can't find the cylindrical component." << endl;
	else
	{
		cout << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << endl;
	}
	//---------------���ӻ���������������ԭʼ���ƣ�ƽ�棬Բ��------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("segment display"));
	//ԭʼ����
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.33, 1, v1);
	viewer->setBackgroundColor(0, 255, 0, v1);
	viewer->addPointCloud(cloud, "cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	//ƽ��
	int v2(0);
	viewer->createViewPort(0.33, 0.0, 0.66, 1, v2);
	viewer->setBackgroundColor(0, 0, 255, v2);
	viewer->addPointCloud(cloud_plane, "cloud_plane", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_plane");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_plane");
	//Բ��
	int v3(0);
	viewer->createViewPort(0.66, 0.0, 1, 1, v3);
	viewer->setBackgroundColor(0, 255, 0, v3);
	viewer->addPointCloud(cloud_cylinder, "cloud_cylinder", v3);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_cylinder");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_cylinder");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
	return 0;
}

