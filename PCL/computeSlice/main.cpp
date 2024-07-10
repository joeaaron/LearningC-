#include <iostream>
#include "Slice.h"
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;

#define PNT_STRIDE 12

void input_stl_PNT(const char* name, SliceBuf* pbuf)
{
	short int PNT2;
	float PNT1[80 + 1];
	long int ct;
	FILE* fid;

	if ((fid = fopen(name, "rb")) == NULL) {
		printf("ģ���ļ��򿪴���");   return;
	}
	for (int i = 1; i <= 20; i++)  PNT1[i] = 0;
	fread(&PNT1[1], sizeof(float), 20, fid);
	fread(&ct, sizeof(long int), 1, fid);
	PNT2 = 0;

	//�����ڴ�
	pbuf->pCloud = (float*)malloc(((ct + 1) * PNT_STRIDE + 1) * sizeof(float));
	pbuf->lCloudNum = ct;

	for (int k = 1; k <= ct; k++)
	{
		for (int j = 1; j <= PNT_STRIDE; j++)
			fread(&pbuf->pCloud[k * PNT_STRIDE + j], sizeof(float), 1, fid);
		fread(&PNT2, sizeof(short int), 1, fid);
	}
	fclose(fid);
}

void output_slc_pmt(SliceBuf* pbuf, const char* name)
{
	FILE* fid;
	int i, k, num;
	float x1, y1, z1, x2, y2, z2, x3, y3, z3;
	double distance, intv = 0.03;

	//���ļ�
	if ((fid = fopen(name, "w")) == NULL) {
		printf("�ļ��򿪴���"); exit(0);
	}

	//�����Ƭ���Ƶ��ļ�
	for (k = 1; k <= pbuf->nCnt; k++) {
		x1 = pbuf->slc[k].vec0.x();
		y1 = pbuf->slc[k].vec0.y();
		z1 = pbuf->slc[k].vec0.z();
		x2 = pbuf->slc[k].vec1.x();
		y2 = pbuf->slc[k].vec1.y();
		z2 = pbuf->slc[k].vec1.z();
		//�������Ƿ��㹻С
		distance = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));
		if (distance > intv) {
			num = distance / intv;
			for (i = 1; i <= num; i++) {
				x3 = x1 + (x2 - x1) * i / (num + 1);
				y3 = y1 + (y2 - y1) * i / (num + 1);
				z3 = z1 + (z2 - z1) * i / (num + 1);
				fprintf(fid, "%f  %f  %f\n", x3, y3, z3);
			}
		}
		fprintf(fid, "%f  %f  %f\n", x1, y1, z1);
		fprintf(fid, "%f  %f  %f\n", x2, y2, z2);
	}

	fclose(fid);
}

int main(int argc, char** argv)
{
	//����STL��ʽ
	SliceBuf* buf = new SliceBuf();
	input_stl_PNT("test1.stl", buf);

	PCL_Slice slice;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	slice.Execute(pCloudOut, buf);

	output_slc_pmt(buf, "E:\\test3.asc");

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"��Ƭ����");

	viewer->addPointCloud(pCloudOut, "Slice");               
	viewer->setRepresentationToSurfaceForAllActors();            
	viewer->initCameraParameters();
	viewer->spin();

	return 0;
}

