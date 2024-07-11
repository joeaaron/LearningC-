#include <iostream>
#include "Slice.h"
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;

#define PNT_STRIDE 12

void SLT2Cloud(std::istream& stlStream, SliceBuf* pbuf) 
{
	if (!stlStream) {
		throw std::runtime_error("STL��������Ч");
	}

	// STL�ļ�ͷͨ����80�ֽڣ���������������
	char header[80];
	stlStream.read(header, 80);

	// ��ȡ��Ƭ��
	long int ct;
	stlStream.read(reinterpret_cast<char*>(&ct), sizeof(long int));

	// �����Ƭ���Ƿ���Ч
	if (ct <= 0) {
		throw std::runtime_error("��Ч����Ƭ��");
	}

	// ��̬�����ڴ�
	pbuf->pCloud.resize(ct * PNT_STRIDE);

	// ��ȡÿ����Ƭ������
	for (long int k = 0; k < ct; ++k) {
		// ��ȡ��Ƭ�Ķ�������
		for (int j = 0; j < 3 * 3; ++j) { // ÿ����Ƭ��3�����㣬ÿ��������3������
			float vertex;
			stlStream.read(reinterpret_cast<char*>(&vertex), sizeof(float));
			pbuf->pCloud[k * (3 * 3) + j] = vertex;
		}

		// ������Ƭ�ķ�����
		float normal[3];
		stlStream.read(reinterpret_cast<char*>(normal), sizeof(float) * 3);

		// ��ȡ�����ֽڼ�����ͨ����0��1
		unsigned short attributeByteCount;
		stlStream.read(reinterpret_cast<char*>(&attributeByteCount), sizeof(unsigned short));
		//if (attributeByteCount > 0) {
		//	// ������������
		//	char attributeData[attributeByteCount];
		//	stlStream.read(attributeData, attributeByteCount);
		//}
	}

	// ������Ƭ��
	pbuf->lCloudNum = ct;
}

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
	pbuf->pCloud.resize(ct * PNT_STRIDE * sizeof(float));

	short int PNT2;
	for (long int k = 1; k <= ct; ++k) 
	{
		for (int j = 1; j <= PNT_STRIDE; j++)
			fid.read(reinterpret_cast<char*>(pbuf->pCloud.data() + k * PNT_STRIDE + j), sizeof(float));
		fid.read(reinterpret_cast<char*>(&PNT2), sizeof(short int));
	}

	pbuf->lCloudNum = ct;
	fid.close();
}

int main(int argc, char** argv)
{
	//����STL��ʽ
	SliceBuf* buf = new SliceBuf();
	SLT2Cloud("test1.stl", buf);

	PCL_Slice slice;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	slice.Execute(pCloudOut, buf);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"��Ƭ����");

	viewer->addPointCloud(pCloudOut, "Slice");               
	viewer->setRepresentationToSurfaceForAllActors();            
	viewer->initCameraParameters();
	viewer->spin();

	return 0;
}

