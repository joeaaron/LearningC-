#include <iostream>
#include "InitialAlign.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h> 

using namespace std;

int main()
{
	pcl::PolygonMesh mesh;
	if (pcl::io::loadPolygonFileSTL("model.stl", mesh) == -1)		//Prismatic002.stl model.stl
	{
		PCL_ERROR("STL��ȡʧ�� \n");
		return (-1);
	}

	//Load scene
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudScene(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("scene.pcd", *cloudScene) == -1)  // scene.pcd
	{
		PCL_ERROR("Couldn't read file\n");
		return (-1);
	}
	cout << "���Ƶ�����" << cloudScene->points.size() << endl;

	pcl::console::TicToc time;
	time.tic();

	PCL_InitialAlign iniAlign;
	iniAlign.Execute(mesh, cloudScene);
	
	cout << "�任����Ϊ��\n" << iniAlign.GetTransform() << endl;
	cout << "Ԥ�����㷨��ʱ��" << time.toc() / 1000 << " ��" << endl;
	return 0;
}
