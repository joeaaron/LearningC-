#include <iostream>
#include "Slice.h"
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;

#define PNT_STRIDE 12

void SLT2Cloud(std::istream& stlStream, SliceBuf* pbuf) 
{
	if (!stlStream) {
		throw std::runtime_error("STL数据流无效");
	}

	// STL文件头通常是80字节，这里我们跳过它
	char header[80];
	stlStream.read(header, 80);

	// 读取面片数
	long int ct;
	stlStream.read(reinterpret_cast<char*>(&ct), sizeof(long int));

	// 检查面片数是否有效
	if (ct <= 0) {
		throw std::runtime_error("无效的面片数");
	}

	// 动态分配内存
	pbuf->pCloud.resize(ct * PNT_STRIDE);

	// 读取每个面片的数据
	for (long int k = 0; k < ct; ++k) {
		// 读取面片的顶点坐标
		for (int j = 0; j < 3 * 3; ++j) { // 每个面片有3个顶点，每个顶点有3个坐标
			float vertex;
			stlStream.read(reinterpret_cast<char*>(&vertex), sizeof(float));
			pbuf->pCloud[k * (3 * 3) + j] = vertex;
		}

		// 跳过面片的法向量
		float normal[3];
		stlStream.read(reinterpret_cast<char*>(normal), sizeof(float) * 3);

		// 读取属性字节计数，通常是0或1
		unsigned short attributeByteCount;
		stlStream.read(reinterpret_cast<char*>(&attributeByteCount), sizeof(unsigned short));
		//if (attributeByteCount > 0) {
		//	// 跳过属性数据
		//	char attributeData[attributeByteCount];
		//	stlStream.read(attributeData, attributeByteCount);
		//}
	}

	// 设置面片数
	pbuf->lCloudNum = ct;
}

void SLT2Cloud(const char* name, SliceBuf* pbuf)
{
	std::ifstream fid(name, std::ios::binary);
	if (!fid) {
		std::cerr << "模型文件打开错误" << std::endl;
		return;
	}

	float PNT1[20];
	fid.read(reinterpret_cast<char*>(PNT1), sizeof(float) * 20);

	long int ct;	// 面片数
	fid.read(reinterpret_cast<char*>(&ct), sizeof(long int));

	// 使用vector动态分配内存
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
	//输入STL格式
	SliceBuf* buf = new SliceBuf();
	SLT2Cloud("test1.stl", buf);

	PCL_Slice slice;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	slice.Execute(pCloudOut, buf);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"切片点云");

	viewer->addPointCloud(pCloudOut, "Slice");               
	viewer->setRepresentationToSurfaceForAllActors();            
	viewer->initCameraParameters();
	viewer->spin();

	return 0;
}

