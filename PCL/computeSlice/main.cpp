#include <iostream>
#include "Slice.h"
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;

#define PNT_STRIDE 12

void SLT2Cloud(const char* name, SliceBuf* pbuf)
{
	short int PNT2;
	float PNT1[80 + 1];
	int i, j, k;
	long int ct;
	FILE* fid;

	if ((fid = fopen(name, "rb")) == NULL) {
		printf("模型文件打开错误");   return;
	}
	for (i = 1; i <= 20; i++)  PNT1[i] = 0;
	fread(&PNT1[1], sizeof(float), 20, fid);
	fread(&ct, sizeof(long int), 1, fid);
	PNT2 = 0;

	//申请内存
	//pbuf->pCloud = (float*)malloc(((ct + 1) * 12 + 1) * sizeof(float));
	pbuf->pCloud.resize(((ct + 1) * 12 + 1) * sizeof(float));
	pbuf->lCloudNum = ct;

	for (k = 1; k <= ct; k++) {
		for (j = 1; j <= 12; j++)
			fread(&pbuf->pCloud[k * 12 + j], sizeof(float), 1, fid);
		fread(&PNT2, sizeof(short int), 1, fid);
	}
	//pbuf->pCloud.erase(pbuf->pCloud.begin());
	fclose(fid);
}

int main(int argc, char** argv)
{
	//输入STL格式
	SliceBuf* buf = new SliceBuf();
	SLT2Cloud("test1.stl", buf);

	PCL_Slice slice;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudOut(new pcl::PointCloud<pcl::PointXYZ>);

	//Eigen::Vector3d anchorPt(-8.891, 0, 0.0);		 // X
	//Eigen::Vector3d anchorPt(0, 1.109, 0.0);		 // Y
	//Eigen::Vector3d anchorPt(0, 0, 9.903);		 // Z
	//Eigen::Vector3d anchorPt(32.854, -2.605, 21.810);		 // Any--取中心点
	//Eigen::Vector3d anchorPt(-18.605, 7.795, 9.999);		 // Any--锚定位置  n(1, 6, 1);
	//Eigen::Vector3d anchorPt(72.308, 20.916, 9.999);		 // Any--锚定位置  n(3, 6, 1);
	//Eigen::Vector3d anchorPt(15.950, -0.744, 10.000);		 // Any--锚定位置  n(1, 2, 3);
	//Eigen::Vector3d anchorPt(70.381, -46.068, 0.000);		 // Any--锚定位置  n(1, 1, 1);
	Eigen::Vector3d anchorPt(17.375, 0.056, 8.992);		 // Any--锚定位置  n(0.267, 0.535, 0.802);
	// X轴切割
	//Eigen::AngleAxisd rotation(-M_PI / 2, Eigen::Vector3d::UnitY());
	//Eigen::Matrix3d rotationMatrix = rotation.toRotationMatrix();
	//Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(0, 1, 2);		// XYZ顺序
	//Eigen::Vector3d rotatedPt = rotationMatrix * anchorPt;

	// Y轴切割
	//Eigen::AngleAxisd rotation(M_PI / 2, Eigen::Vector3d::UnitX());
	//Eigen::Matrix3d rotationMatrix = rotation.toRotationMatrix();
	//Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(0, 1, 2);		// XYZ顺序
	//Eigen::Vector3d rotatedPt = rotationMatrix * anchorPt;
	
	// Z轴切割
	//Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
	//Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(0, 1, 2);		// XYZ顺序
	//Eigen::Vector3d rotatedPt = rotationMatrix * anchorPt;

	// 任意平面
	Eigen::Vector3d n(0.267, 0.535, 0.802);
	Eigen::Vector3d z(0, 0, 1);

	Eigen::Vector3d axis = n.cross(z);
	double angle = acos(n.dot(z)/ n.norm());

	Eigen::AngleAxisd rotation(angle, axis.normalized());
	Eigen::Matrix3d rotationMatrix = rotation.toRotationMatrix();
	Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(0, 1, 2);		// XYZ顺序
	Eigen::Vector3d rotatedPt = rotationMatrix * anchorPt;

	// 设置变换
	TransData data(eulerAngles[0], eulerAngles[1], eulerAngles[2], 0, 0, 0);
	slice.SetTransData(data);

	slice.SetSlicePos(rotatedPt[2]);
	slice.Execute(pCloudOut, buf);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"切片点云");

	viewer->addPointCloud(pCloudOut, "Slice");               
	viewer->setRepresentationToSurfaceForAllActors();            
	viewer->initCameraParameters();
	viewer->resetCamera();
	viewer->spin();

	return 0;
}

