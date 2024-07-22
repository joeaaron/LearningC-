#pragma warning(disable:4996)

#include "windows.h"
#include "math.h"  //数学函数库
#include "stdio.h"  //输入输出库
#include "string.h"  //字符库
#include "stdlib.h"  //流函数库

#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <iostream>
#include <cmath>

#undef min
#undef max

#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>

using namespace Eigen;

#define PNT_MAX  60000
#define ROT_MATRIX_SIZE 3
#define TRANSLATION_SIZE 3
#define PNT_STRIDE 12

// 假设存在一个结构体定义如下，用于存储线段的属性
typedef struct LineSegment 
{
	double x, y, z;					// 线段端点的坐标
	double r = 0, g = 0, b = 0;		// 线段端点的颜色

	LineSegment() {}
	LineSegment(double dx, double dy, double dz, double dr, double dg, double db)
	{
		x = dx;
		y = dy;
		z = dz;
		r = dr;
		g = dg;
		b = db;
	}

	LineSegment& operator=(const LineSegment& rhs)
	{
		if (this == &rhs) return *this;
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		r = rhs.r;
		g = rhs.g;
		b = rhs.b;
		
		return *this;
	}

} LineSegment;

struct type_buf 
{
	float* PNT;    //存放点云
	long int ct;   //原始点云计数
	int cnt;	   //切片点云计数

	struct {
		LineSegment lineSeg0;
		LineSegment lineSeg1;
		int line_m, line_n;
		float x0, y0, z0;		// 切面端点0
		float x1, y1, z1;		// 切面端点1
	}slc[PNT_MAX];  //切片点云
} buf;

// 假设inVar是某种结构体，包含了旋转角度和平移值
struct InVar 
{
	double rot_x, rot_y, rot_z;
	double move_x, move_y, move_z;

	Eigen::Matrix4d mtxR;
	Eigen::Vector4d vecT;
}inVar;

void get_var(void);
void input_stl_PNT(const char*, struct type_buf*);
void compute_slc(double, struct type_buf*);
void output_slc_pmt(struct type_buf*, const char* name);

//***************************************************************
//输入参数
void get_var(void)
{
	//平移量
	inVar.move_x = 0;
	inVar.move_y = 0;
	inVar.move_z = 0;
	//旋转量
	inVar.rot_x = 0;
	inVar.rot_y = 0;
	inVar.rot_z = 0;
}

//***************************************************************************
//输入点云
void input_stl_PNT(const char* name, struct type_buf* pbuf)
{
	short int PNT2;
	float PNT1[80 + 1];
	long int ct;
	FILE* fid;

	if ((fid = fopen(name, "rb")) == NULL) {
		printf("模型文件打开错误");   return;
	}
	for (int i = 1; i <= 20; i++)  PNT1[i] = 0;
	fread(&PNT1[1], sizeof(float), 20, fid);
	fread(&ct, sizeof(long int), 1, fid);
	PNT2 = 0;

	//申请内存
	pbuf->PNT = (float*)malloc(((ct + 1) * PNT_STRIDE + 1) * sizeof(float));
	pbuf->ct = ct;

	for (int k = 1; k <= ct; k++) 
	{
		for (int j = 1; j <= PNT_STRIDE; j++)
			fread(&pbuf->PNT[k * PNT_STRIDE + j], sizeof(float), 1, fid);
		fread(&PNT2, sizeof(short int), 1, fid);
	}
	fclose(fid);
}

void RotatePnt(struct type_buf* pbuf)
{
	int ct = pbuf->ct;
	auto PNT = pbuf->PNT;

	// 摆放旋转平移
	double sin_x = sin(inVar.rot_x * M_PI / 180);
	double cos_x = cos(inVar.rot_x * M_PI / 180);

	// 定义旋转矩阵 R1, R2, R3
	Matrix3d R1, R2, R3;
	R1 << 1, 0, 0,
		0, cos_x, -sin_x,
		0, sin_x, cos_x;

	R2 << cos_x, 0, sin_x,
		0, 1, 0,
		-sin_x, 0, cos_x;

	R3 << cos_x, -sin_x, 0,
		sin_x, cos_x, 0,
		0, 0, 1;

	// 计算 R4 = R2 * R1, R5 = R3 * R4
	Matrix3d R4 = R2 * R1;
	Matrix3d R5 = R3 * R4;

	// 平移向量
	Vector3d T5(inVar.move_x, inVar.move_y, inVar.move_z);

	// 旋转和平移点云
	for (int kk = 0; kk < ct; kk++) 
	{
		for (int i = 0; i < 4; i++) 
		{
			Vector3d p;
			for (int j = 0; j < 3; j++) 
			{
				p[j] = PNT[kk * PNT_STRIDE + i * 3 + j];
			}

			// 点云旋转平移后的坐标 p2 = R5 * p1 + T5
			Vector3d transformed_p = R5 * p;
			if (i > 0) 
			{
				transformed_p += T5;
			}

			for (int j = 0; j < 3; j++) {
				PNT[kk * PNT_STRIDE + i * 3 + j] = transformed_p[j];
			}
		}
	}

	// 记录旋转矩阵和平移向量
	Eigen::Matrix4d mat;
	mat << 0, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	inVar.mtxR = mat;

	Eigen::Vector4d vec;
	vec.head<3>() = T5;
	vec[3] = 0;
	inVar.vecT = vec;
}

// 函数用于计算线段的端点坐标和颜色
void CalLineSeg(LineSegment& lineSegOut, double z9, const LineSegment& lineSeg1, const LineSegment& lineSeg2)
{
	double dCoef1 = z9 - lineSeg1.z;
	double dCoef2 = lineSeg2.z - lineSeg1.z;

	lineSegOut.x = dCoef1 * (lineSeg2.x - lineSeg1.x) / dCoef2 + lineSeg1.x;
	lineSegOut.y = dCoef1 * (lineSeg2.y - lineSeg1.y) / dCoef2 + lineSeg1.y;
	lineSegOut.r = dCoef1 * (lineSeg2.r - lineSeg1.r) / dCoef2 + lineSeg1.r;
	lineSegOut.g = dCoef1 * (lineSeg2.g - lineSeg1.g) / dCoef2 + lineSeg1.g;
	lineSegOut.b = dCoef1 * (lineSeg2.b - lineSeg1.b) / dCoef2 + lineSeg1.b;
}

// 两个线段具有相同的两个端点
bool IsSegmentEqual(const LineSegment& lineSeg1, const LineSegment& lineSeg2,
	const LineSegment& slcLineSeg0, const LineSegment& slcLineSeg1)
{
	return (lineSeg1.x == slcLineSeg0.x && lineSeg2.x == slcLineSeg1.x && lineSeg1.y == slcLineSeg0.y && lineSeg2.y == slcLineSeg1.y)
		|| (lineSeg2.x == slcLineSeg0.x && lineSeg1.x == slcLineSeg1.x && lineSeg2.y == slcLineSeg0.y && lineSeg1.y == slcLineSeg1.y);
}
//******************************************************************
/**
 * @brief 计算切片
 *
 * 根据给定的 z 值（z9）和缓冲区（pbuf）中的数据，计算切片并存储在 pbuf 的 slc 数组中。
 *
 * @param z9 切片的 z 值
 * @param pbuf 缓冲区结构体指针，包含点坐标和其他信息
 */
void ComputeSlc(double z9, struct type_buf* pbuf)
{
	auto PNT = pbuf->PNT;
	int ct = pbuf->ct;
	int cnt = 0;

	LineSegment lineSeg1;
	LineSegment lineSeg2;
	LineSegment lineSeg3;

	//计算切片
	for (int k = 1; k <= ct && cnt + 1 < PNT_MAX; k++)
	{
		// 提取当前点和相邻两点的坐标
		double xn = pbuf->PNT[k * PNT_STRIDE + 1];
		double yn = pbuf->PNT[k * PNT_STRIDE + 2];
		double zn = pbuf->PNT[k * PNT_STRIDE + 3];

		lineSeg1.x = PNT[k * PNT_STRIDE + 4]; lineSeg1.y = PNT[k * PNT_STRIDE + 5]; lineSeg1.z = PNT[k * PNT_STRIDE + 6];
		lineSeg2.x = PNT[k * PNT_STRIDE + 7]; lineSeg2.y = PNT[k * PNT_STRIDE + 8]; lineSeg2.z = PNT[k * PNT_STRIDE + 9];
		lineSeg3.x = PNT[k * PNT_STRIDE + 10]; lineSeg3.y = PNT[k * PNT_STRIDE + 11]; lineSeg3.z = PNT[k * PNT_STRIDE + PNT_STRIDE];

		//查找位置相临近的点坐标
		if (lineSeg1.z == z9 && lineSeg2.z == z9 && lineSeg3.z == z9) { continue; }
		bool flagRelated = false;
		int dt = 0;

		// 如果三个相邻点都在z9平面的同侧（即都大于或都小于z9），则进一步检查是否有一个或两个点在z9平面上。
		// 如果是，则将这些线段存储在pbuf->slc中。
		if ((lineSeg1.z >= z9 && lineSeg2.z >= z9 && lineSeg3.z >= z9) 
			|| (lineSeg1.z <= z9 && lineSeg2.z <= z9 && lineSeg3.z <= z9))
		{	
			if (lineSeg1.z == z9 && lineSeg2.z == z9 && lineSeg3.z > z9)
			{
				cnt = cnt + 1;  dt = 0;   flagRelated = true;
				pbuf->slc[cnt].lineSeg0 = lineSeg1;
				dt++;
				pbuf->slc[cnt].lineSeg1 = lineSeg2;
				dt++;
					
			}
			if (lineSeg1.z == z9 && lineSeg3.z == z9 && lineSeg2.z > z9)
			{
				cnt = cnt + 1;  dt = 0;   flagRelated = true;
				pbuf->slc[cnt].lineSeg0 = lineSeg1;
				dt++;
				pbuf->slc[cnt].lineSeg1 = lineSeg3;
				dt++;
			}
			if (lineSeg3.z == z9 && lineSeg2.z == z9 && lineSeg1.z > z9)
			{
				cnt = cnt + 1;  dt = 0;   flagRelated = true;
				pbuf->slc[cnt].lineSeg0 = lineSeg3;
				dt++;
				pbuf->slc[cnt].lineSeg1 = lineSeg2;
				dt++;
			}
			pbuf->slc[cnt].line_m = 2;
		}
		// 如果三个相邻点不在z9平面的同侧，则计算与z9平面相交的线段，并将其存储在pbuf->slc中。
		else 
		{  
			//对点进行比对后的删除
			cnt = cnt + 1;  dt = 0;   flagRelated = true;
			LineSegment lineSegOut;

			if ((lineSeg1.z - z9) * (lineSeg2.z - z9) < 0)
			{
				CalLineSeg(lineSegOut, z9, lineSeg1, lineSeg2);
			
				if (dt == 0) { pbuf->slc[cnt].lineSeg0 = lineSegOut; }
				if (dt == 1) { pbuf->slc[cnt].lineSeg1 = lineSegOut; }

				dt++;
			}
			if ((lineSeg1.z - z9) * (lineSeg3.z - z9) < 0)
			{
				CalLineSeg(lineSegOut, z9, lineSeg1, lineSeg3);

				if (dt == 0) { pbuf->slc[cnt].lineSeg0 = lineSegOut; }
				if (dt == 1) { pbuf->slc[cnt].lineSeg1 = lineSegOut; }

				dt++;
			}
			if ((lineSeg3.z - z9) * (lineSeg2.z - z9) < 0)
			{		
				CalLineSeg(lineSegOut, z9, lineSeg3, lineSeg2);

				if (dt == 0) { pbuf->slc[cnt].lineSeg0 = lineSegOut; }
				if (dt == 1) { pbuf->slc[cnt].lineSeg1 = lineSegOut; }

				dt++;
			}
			if (lineSeg1.z == z9 && (lineSeg3.z - z9) * (lineSeg2.z - z9) < 0)
			{
				if (dt == 0) { pbuf->slc[cnt].lineSeg0 = lineSeg1; }
				if (dt == 1) { pbuf->slc[cnt].lineSeg1 = lineSeg1; }

				dt++;
			}
			if (lineSeg2.z == z9 && (lineSeg1.z - z9) * (lineSeg3.z - z9) < 0)
			{
				if (dt == 0) { pbuf->slc[cnt].lineSeg0 = lineSeg2; }
				if (dt == 1) { pbuf->slc[cnt].lineSeg1 = lineSeg2; }

				dt++;
			}
			if (lineSeg3.z == z9 && (lineSeg1.z - z9) * (lineSeg2.z - z9) < 0)
			{
				if (dt == 0) { pbuf->slc[cnt].lineSeg0 = lineSeg3; }
				if (dt == 1) { pbuf->slc[cnt].lineSeg1 = lineSeg3; }

				dt++;
			}

			pbuf->slc[cnt].line_m = 1;
			/*if (dt < 2)   flagSameSide = true;*/
		}

		//计算切片点位置：对于每个与z9平面相交的线段，确定其在切片平面上的方向（基于当前点和线段的方向）。
		if (flagRelated)
		{
			lineSeg1 = pbuf->slc[cnt].lineSeg0;
			lineSeg2 = pbuf->slc[cnt].lineSeg1;
			lineSeg3.x = lineSeg2.x - lineSeg1.x;     
			lineSeg3.y = lineSeg2.y - lineSeg1.y;

			if (xn * lineSeg3.y - yn * lineSeg3.x < 0)
			{
				pbuf->slc[cnt].line_n = -1;
			}	
			else
			{
				pbuf->slc[cnt].line_n = 1;
			}
		}
	}

	//删除buf.line_m[cnt]=2;重复线段（都在上侧，两段线都删除）
	//切线的两三角形在切平面的：1上下，2同侧，3一上一平，4两平    
	//上下选z>z9的三角形 
	for (int k = 1; k <= cnt - 1; k++)
	{
		if (pbuf->slc[k].line_m == 2)
		{
			bool flagSameSide = false;
			lineSeg1 = pbuf->slc[k].lineSeg0;
			lineSeg2 = pbuf->slc[k].lineSeg1;

			for (int i = k + 1; i <= cnt; i++)
			{		
				if (IsSegmentEqual(lineSeg1, lineSeg2, pbuf->slc[i].lineSeg0, pbuf->slc[i].lineSeg1))
				{
					flagSameSide = true;

					for (int j = i; j <= cnt - 1; j++)
					{
						pbuf->slc[j] = pbuf->slc[j + 1]; // 向前移动线段
					}
					--cnt; // 减少计数
				}
			}
			//同侧的两个线段都删除
			if (flagSameSide)
			{
				for (int j = k; j <= cnt - 1; j++)
				{
					pbuf->slc[j] = pbuf->slc[j + 1]; // 向前移动线段
				}
				--cnt;
			}
		}
	}

	// 更新pbuf->cnt以反映存储在pbuf->slc中的线段数量
	pbuf->cnt = cnt;
}

/**
 * @brief 旋转 SLC 切片点
 *
 * 根据给定的旋转矩阵和平移向量，对 SLC 切片中的点进行旋转和平移操作。
 *
 * @param z 旋转平面的 Z 坐标
 * @param pbuf SLC 切片数据的指针
 */
void RotateSlcPnt(double z, struct type_buf* pbuf)
{
	Eigen::Matrix3d R;
	Eigen::Vector3d T;

	// 旋转矩阵
	for (int i = 0; i < ROT_MATRIX_SIZE; i++) 
	{
		for (int j = 0; j < ROT_MATRIX_SIZE; j++) 
		{
			R(i, j) = inVar.mtxR(i + 1, j + 1); 
		}
		T(i) = inVar.vecT(i + 1); 
	}

	// 辅助函数，对给定的点进行旋转和平移
	auto rotateAndTranslate = [&](double x, double y, double& newX, double& newY, double& newZ) {
		Eigen::Vector3d p(x, y, z);
		Eigen::Vector3d q = R * (p - T) + T;
		newX = q.x();
		newY = q.y();
		newZ = q.z();
		};

	// 对切片端点0和1进行旋转和平移
	for (int k = 0; k < pbuf->cnt; k++) 
	{
		double newX0, newY0, newZ0;
		double newX1, newY1, newZ1;

		rotateAndTranslate(pbuf->slc[k].lineSeg0.x, pbuf->slc[k].lineSeg0.y, newX0, newY0, newZ0);
		pbuf->slc[k].x0 = static_cast<float>(newX0);
		pbuf->slc[k].y0 = static_cast<float>(newY0);
		pbuf->slc[k].z0 = static_cast<float>(newZ0);

		rotateAndTranslate(pbuf->slc[k].lineSeg1.x, pbuf->slc[k].lineSeg1.y, newX1, newY1, newZ1);
		pbuf->slc[k].x1 = static_cast<float>(newX1);
		pbuf->slc[k].y1 = static_cast<float>(newY1);
		pbuf->slc[k].z1 = static_cast<float>(newZ1);
	}
}

//************************************************************
//输出点云
void output_slc_pmt(struct type_buf* pbuf, const char* name)
{
	FILE* fid;
	int i, k, num;
	float x1, y1, z1, x2, y2, z2, x3, y3, z3;
	double distance, intv = 0.03;

	//打开文件
	if ((fid = fopen(name, "w")) == NULL) {
		printf("文件打开错误"); exit(0);
	}

	//输出切片点云到文件
	for (k = 1; k <= pbuf->cnt; k++) {
		x1 = pbuf->slc[k].x0;
		y1 = pbuf->slc[k].y0;
		z1 = pbuf->slc[k].z0;
		x2 = pbuf->slc[k].x1;
		y2 = pbuf->slc[k].y1;
		z2 = pbuf->slc[k].z1;
		//检测距离是否足够小
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

//***************************************************************************
//主函数
void main(void)
{
	//输入参数
	get_var();

	// 创建一个pcl::PolygonMesh对象
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

	// 加载STL文件
	if (pcl::io::loadPolygonFileSTL("test1.stl", *mesh) == -1) {
		return ;
	}

	//输入STL格式
	input_stl_PNT("test1.stl", &buf);

	//旋转点云
	RotatePnt(&buf);

	//切片位置
	double z = 6;

	//切片结果到buf
	ComputeSlc(z, &buf);

	//旋转切片结果
	RotateSlcPnt(z, &buf);

	//输出点云
	output_slc_pmt(&buf, "E:\\test3.asc");

	//释放内存
	//free(buf.PNT);
}


