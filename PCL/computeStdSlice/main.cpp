#include <iostream>
#include "Slice.h"
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;

#define PNT_STRIDE 12

void SLT2Cloud(const char* name, SliceBuf* pbuf)
{
	std::ifstream fid(name, std::ios::binary);
	if (!fid) {
		std::cerr << "ģ���ļ��򿪴���" << std::endl;
		return;
	}

	float PNT1[20];
	fid.read(reinterpret_cast<char*>(PNT1), sizeof(float) * 20);

	long int ct;	// ��Ƭ��
	fid.read(reinterpret_cast<char*>(&ct), sizeof(long int));

	// ʹ��vector��̬�����ڴ�
	pbuf->vCloud.resize(ct * PNT_STRIDE * sizeof(float));

	short int PNT2;
	for (long int k = 1; k <= ct; ++k) 
	{
		for (int j = 1; j <= PNT_STRIDE; j++)
			fid.read(reinterpret_cast<char*>(pbuf->vCloud.data() + k * PNT_STRIDE + j), sizeof(float));
		fid.read(reinterpret_cast<char*>(&PNT2), sizeof(short int));
	}

	pbuf->lCloudNum = ct;
	fid.close();
}

int main(int argc, char** argv)
{
	//����STL��ʽ
	SliceBuf* buf = new SliceBuf();
	SLT2Cloud("cad.stl", buf);

	PCL_Slice slice;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	slice.SetAxisDir(em3DCloudSliceAxisDir::eAxixX);  // ��X��
	slice.SetSlicePos(-12.64);
	slice.Execute(pCloudOut, buf);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"��Ƭ����");

	viewer->addPointCloud(pCloudOut, "Slice");               
	viewer->setRepresentationToSurfaceForAllActors();            
	viewer->initCameraParameters();
	viewer->spin();

	return 0;
}

