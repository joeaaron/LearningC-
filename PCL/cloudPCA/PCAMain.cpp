#include <string>
#include <iostream>
#include "PCARegistration.h"
#include <pcl/console/time.h>			// ����̨����ʱ��
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

using namespace std;
const double LEAF_SIZE = 1.0;

#define ENABLE_DISPLAY 1

bool Mesh2CloudPCL(pcl::PointCloud<pcl::PointXYZ>& cloudOut,
	const pcl::PolygonMesh& mesh)
{
	pcl::fromPCLPointCloud2(mesh.cloud, cloudOut);
	return true;
}

int main(int argc, char** argv)
{
	pcl::console::TicToc time;
	//----------------���ص�������-------------------
	pcl::PolygonMesh mesh;
	if (pcl::io::loadPolygonFileSTL("model.stl", mesh) == -1)					//Prismatic002.stl model.stl model.stl cad.stl
	{
		PCL_ERROR("STL��ȡʧ�� \n");
		return (-1);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudModel(new pcl::PointCloud<pcl::PointXYZ>);
	Mesh2CloudPCL(*cloudModel, mesh);

	//Load scene
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudScene(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("scene.pcd", *cloudScene) == -1)	//scene.pcd
	{
		PCL_ERROR("Couldn't read file\n");
		return (-1);
	}

	cout << "���Ƶ�����" << cloudScene->points.size() << endl;
	time.tic();

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
	vg.setDownsampleAllData(false);

	vg.setInputCloud(cloudModel);
	vg.filter(*cloudModel);

	vg.setInputCloud(cloudScene);
	vg.filter(*cloudScene);

	//--------------PCA����任����------------------
	Eigen::Matrix4f PCATransform = Eigen::Matrix4f::Identity();
	PCATransform = PCARegistration(cloudScene, cloudModel);

	cout << "PCA����任������ʱ�� " << time.toc() / 1000 << " s" << endl;
	cout << "�任����Ϊ��\n" << PCATransform << endl;

	//Eigen::Matrix4f rotation_4d;
	//rotation_4d << -1, 0, 0, 0,
	//	0, 1, 0, 0,
	//	0, 0, -1, 0,
	//	0, 0, 0, 1;

	////PCATransform *= rotation_4d;

	//-----------------�����׼----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCARegisted(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*cloudScene, *PCARegisted, PCATransform);

	//----------------���ӻ����---------------------
	visualize_registration(cloudScene, cloudModel, PCARegisted);
	
	//���о�ȷ��׼������ICP�㷨
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//�������׼���ƺ�Ŀ�����
	icp.setInputSource(cloudModel);
	icp.setInputTarget(cloudScene);
	//Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance(40);
	//����������
	icp.setMaximumIterations(50);
	//���α仯����֮��Ĳ�ֵ
	icp.setTransformationEpsilon(1e-10);
	// �������
	icp.setEuclideanFitnessEpsilon(0.002);
	icp.align(*icp_result, PCATransform);
	Eigen::Matrix4f icp_trans;
	icp_trans = icp.getFinalTransformation();
	std::cout << "icp�任����" << endl << icp_trans << endl;
	std::cout << "icp score:" << icp.getFitnessScore() << endl;

#if ENABLE_DISPLAY
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*cloudScene, *cloud_output, icp_trans);		  // �����϶����ĵ���	

	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer1(new pcl::visualization::PCLVisualizer(u8"���������"));
	viewer1->setBackgroundColor(255, 255, 255);

	// ��Ŀ�������ɫ���ӻ� (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(cloudModel, 255, 0, 0);
	viewer1->addPointCloud<pcl::PointXYZ>(cloudModel, target_color, "target cloud");
	viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");

	// ��Դ������ɫ���ӻ� (green).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>input_color(cloudScene, 0, 255, 0);
	viewer1->addPointCloud<pcl::PointXYZ>(cloud_output, input_color, "input cloud");
	viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input cloud");

	while (!viewer1->wasStopped())
	{
		viewer1->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif

	return (0);
}
