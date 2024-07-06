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

using namespace Eigen;

#define PNT_MAX  60000
#define ROT_MATRIX_SIZE 3
#define TRANSLATION_SIZE 3

// 假设存在一个结构体定义如下，用于存储线段的属性
typedef struct {
	double x, y;		// 线段端点的坐标
	double r, g, b;		// 线段端点的颜色
} LineSegment;

struct type_buf 
{
	float* PNT;    //存放点云
	long int ct;   //点云计数
	int cnt;

	struct {
		LineSegment lineSeg0;
		LineSegment lineSeg1;
		int line_m, line_n;
		float x0, y0, z0;
		float x1, y1, z1;
	}slc[PNT_MAX];  //切片点云
} buf;

// 假设in_var是某种结构体，包含了旋转角度和平移值
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
	pbuf->PNT = (float*)malloc(((ct + 1) * 12 + 1) * sizeof(float));
	pbuf->ct = ct;

	for (k = 1; k <= ct; k++) {
		for (j = 1; j <= 12; j++)
			fread(&pbuf->PNT[k * 12 + j], sizeof(float), 1, fid);
		fread(&PNT2, sizeof(short int), 1, fid);
	}
	fclose(fid);
}

void RotatePnt(struct type_buf* pbuf)
{
	int ct = pbuf->ct;
	float* PNT = pbuf->PNT;

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
				p[j] = PNT[kk * 12 + i * 3 + j];
			}

			// 点云旋转平移后的坐标 p2 = R5 * p1 + T5
			Vector3d transformed_p = R5 * p;
			if (i > 0) 
			{
				transformed_p += T5;
			}

			for (int j = 0; j < 3; j++) {
				PNT[kk * 12 + i * 3 + j] = transformed_p[j];
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

//******************************************************************
//计算切片
void compute_slc(double z9, struct type_buf* pbuf)
{
	int i, j, k, ct, cnt, dt, flag = 0, flag2 = 0, flagc;
	float* PNT;

	double x1, y1, z1;
	double x2, y2, z2; 
	double x3, y3, z3; 
	double x4, y4;
	double xn, yn, zn;
	double x9, y9;
	
	double r, g, b;
	double r1, g1, b1; 
	double r2, g2, b2;
	double r3, g3, b3;

	PNT = pbuf->PNT;
	ct = pbuf->ct;
	cnt = 0;

	//计算切片
	for (k = 1; k <= ct; k++)
	{
		if (cnt + 1 < PNT_MAX) 
		{
			xn = PNT[k * 12 + 1]; yn = PNT[k * 12 + 2]; zn = PNT[k * 12 + 3];
			x1 = PNT[k * 12 + 4]; y1 = PNT[k * 12 + 5]; z1 = PNT[k * 12 + 6];
			x2 = PNT[k * 12 + 7]; y2 = PNT[k * 12 + 8]; z2 = PNT[k * 12 + 9];
			x3 = PNT[k * 12 + 10]; y3 = PNT[k * 12 + 11]; z3 = PNT[k * 12 + 12];

			r1 = 0; g1 = 0; b1 = 0; 
			r2 = 0; g2 = 0; b2 = 0; 
			r3 = 0; g3 = 0; b3 = 0;

			//查找位置相临近的点坐标
			flagc = 0;
			if ((z1 >= z9 && z2 >= z9 && z3 >= z9) || (z1 <= z9 && z2 <= z9 && z3 <= z9)) 
			{
				if (z1 == z9 && z2 == z9 && z3 == z9) {}
				else 
				{
					if (z1 == z9 && z2 == z9 && z3 > z9)
					{
						cnt = cnt + 1;  dt = 0;   flagc = 1;

						// 计算线段的端点坐标和颜色
						pbuf->slc[cnt].lineSeg0.x = x1;   pbuf->slc[cnt].lineSeg0.y = y1;
						pbuf->slc[cnt].lineSeg0.r = r1;   pbuf->slc[cnt].lineSeg0.g = g1;   pbuf->slc[cnt].lineSeg0.b = b1;		dt = dt + 1;
						pbuf->slc[cnt].lineSeg1.x = x2;   pbuf->slc[cnt].lineSeg1.y = y2;
						pbuf->slc[cnt].lineSeg1.r = r2;   pbuf->slc[cnt].lineSeg1.g = g2;   pbuf->slc[cnt].lineSeg1.b = b2;		dt = dt + 1;
					
					}
					if (z1 == z9 && z3 == z9 && z2 > z9) 
					{
						cnt = cnt + 1;  dt = 0;   flagc = 1;
						pbuf->slc[cnt].lineSeg0.x = x1;   pbuf->slc[cnt].lineSeg0.y = y1;
						pbuf->slc[cnt].lineSeg0.r = r1;   pbuf->slc[cnt].lineSeg0.g = g1;   pbuf->slc[cnt].lineSeg0.b = b1;		dt = dt + 1;
						pbuf->slc[cnt].lineSeg1.x = x3;   pbuf->slc[cnt].lineSeg1.y = y3;
						pbuf->slc[cnt].lineSeg1.r = r3;   pbuf->slc[cnt].lineSeg1.g = g3;   pbuf->slc[cnt].lineSeg1.b = b3;		dt = dt + 1;
					}
					if (z3 == z9 && z2 == z9 && z1 > z9) 
					{
						cnt = cnt + 1;  dt = 0;   flagc = 1;
						pbuf->slc[cnt].lineSeg0.x = x3;   pbuf->slc[cnt].lineSeg0.y = y3;
						pbuf->slc[cnt].lineSeg0.r = r3;   pbuf->slc[cnt].lineSeg0.g = g3;   pbuf->slc[cnt].lineSeg0.b = b3;		dt = dt + 1;
						pbuf->slc[cnt].lineSeg1.x = x2;   pbuf->slc[cnt].lineSeg1.y = y2;
						pbuf->slc[cnt].lineSeg1.r = r2;   pbuf->slc[cnt].lineSeg1.g = g2;   pbuf->slc[cnt].lineSeg1.b = b2;		dt = dt + 1;
					}
					pbuf->slc[cnt].line_m = 2;
				}//else

			}
			else 
			{  //对点进行比对后的删除
				cnt = cnt + 1;  dt = 0;   flagc = 1;

				if ((z1 - z9) * (z2 - z9) < 0) 
				{
					x9 = (z9 - z1) * (x2 - x1) / (z2 - z1) + x1;
					y9 = (z9 - z1) * (y2 - y1) / (z2 - z1) + y1;
					if (dt == 0) { pbuf->slc[cnt].lineSeg0.x = x9;   pbuf->slc[cnt].lineSeg0.y = y9; }
					if (dt == 1) { pbuf->slc[cnt].lineSeg1.x = x9;   pbuf->slc[cnt].lineSeg1.y = y9; }
					//
					r = (z9 - z1) * (r2 - r1) / (z2 - z1) + r1;
					g = (z9 - z1) * (g2 - g1) / (z2 - z1) + g1;
					b = (z9 - z1) * (b2 - b1) / (z2 - z1) + b1;
					if (dt == 0) { pbuf->slc[cnt].lineSeg0.r = r;   pbuf->slc[cnt].lineSeg0.g = g;   pbuf->slc[cnt].lineSeg0.b = b; }
					if (dt == 1) { pbuf->slc[cnt].lineSeg1.r = r;   pbuf->slc[cnt].lineSeg1.g = g;   pbuf->slc[cnt].lineSeg1.b = b; }
					dt = dt + 1;
				}
				if ((z1 - z9) * (z3 - z9) < 0) 
				{
					x9 = (z9 - z1) * (x3 - x1) / (z3 - z1) + x1;
					y9 = (z9 - z1) * (y3 - y1) / (z3 - z1) + y1;
					if (dt == 0) { pbuf->slc[cnt].lineSeg0.x = x9;   pbuf->slc[cnt].lineSeg0.y = y9; }
					if (dt == 1) { pbuf->slc[cnt].lineSeg1.x = x9;   pbuf->slc[cnt].lineSeg1.y = y9; }
					//
					r = (z9 - z1) * (r3 - r1) / (z3 - z1) + r1;
					g = (z9 - z1) * (g3 - g1) / (z3 - z1) + g1;
					b = (z9 - z1) * (b3 - b1) / (z3 - z1) + b1;
					if (dt == 0) { pbuf->slc[cnt].lineSeg0.r = r;   pbuf->slc[cnt].lineSeg0.g = g;   pbuf->slc[cnt].lineSeg0.b = b; }
					if (dt == 1) { pbuf->slc[cnt].lineSeg1.r = r;   pbuf->slc[cnt].lineSeg1.g = g;   pbuf->slc[cnt].lineSeg1.g = b; }
					dt = dt + 1;
				}
				if ((z3 - z9) * (z2 - z9) < 0) 
				{
					x9 = (z9 - z3) * (x2 - x3) / (z2 - z3) + x3;
					y9 = (z9 - z3) * (y2 - y3) / (z2 - z3) + y3;
					if (dt == 0) { pbuf->slc[cnt].lineSeg0.x = x9;   pbuf->slc[cnt].lineSeg0.y = y9; }
					if (dt == 1) { pbuf->slc[cnt].lineSeg1.x = x9;   pbuf->slc[cnt].lineSeg1.y = y9; }
					//
					r = (z9 - z3) * (r2 - r3) / (z2 - z3) + r3;
					g = (z9 - z3) * (g2 - g3) / (z2 - z3) + g3;
					b = (z9 - z3) * (b2 - b3) / (z2 - z3) + b3;
					if (dt == 0) { pbuf->slc[cnt].lineSeg0.r = r;   pbuf->slc[cnt].lineSeg0.g = g;   pbuf->slc[cnt].lineSeg0.b = b; }
					if (dt == 1) { pbuf->slc[cnt].lineSeg1.r = r;   pbuf->slc[cnt].lineSeg1.g = g;   pbuf->slc[cnt].lineSeg1.b = b; }
					dt = dt + 1;
				}
				if (z1 == z9 && (z3 - z9) * (z2 - z9) < 0) 
				{
					if (dt == 0) { pbuf->slc[cnt].lineSeg0.x = x1;   pbuf->slc[cnt].lineSeg0.y = y1; }
					if (dt == 1) { pbuf->slc[cnt].lineSeg1.x = x1;   pbuf->slc[cnt].lineSeg1.y = y1; }
					if (dt == 0) { pbuf->slc[cnt].lineSeg0.r = r1;   pbuf->slc[cnt].lineSeg0.g = g1;   pbuf->slc[cnt].lineSeg0.b = b1; }
					if (dt == 1) { pbuf->slc[cnt].lineSeg1.r = r1;   pbuf->slc[cnt].lineSeg1.g = g1;   pbuf->slc[cnt].lineSeg1.b = b1; }
					dt = dt + 1;
				}
				if (z2 == z9 && (z1 - z9) * (z3 - z9) < 0) 
				{
					if (dt == 0) { pbuf->slc[cnt].lineSeg0.x = x2;   pbuf->slc[cnt].lineSeg0.y = y2; }
					if (dt == 1) { pbuf->slc[cnt].lineSeg1.x = x2;   pbuf->slc[cnt].lineSeg1.y = y2; }
					if (dt == 0) { pbuf->slc[cnt].lineSeg0.r = r2;   pbuf->slc[cnt].lineSeg0.g = g2;   pbuf->slc[cnt].lineSeg0.b = b2; }
					if (dt == 1) { pbuf->slc[cnt].lineSeg1.r = r2;   pbuf->slc[cnt].lineSeg1.g = g2;   pbuf->slc[cnt].lineSeg1.b = b2; }
					dt = dt + 1;
				}
				if (z3 == z9 && (z1 - z9) * (z2 - z9) < 0) 
				{
					if (dt == 0) { pbuf->slc[cnt].lineSeg0.x = x3;   pbuf->slc[cnt].lineSeg0.y = y3; }
					if (dt == 1) { pbuf->slc[cnt].lineSeg1.x = x3;   pbuf->slc[cnt].lineSeg1.y = y3; }
					if (dt == 0) { pbuf->slc[cnt].lineSeg0.r = r3;   pbuf->slc[cnt].lineSeg0.g = g3;   pbuf->slc[cnt].lineSeg0.b = b3; }
					if (dt == 1) { pbuf->slc[cnt].lineSeg1.r = r3;   pbuf->slc[cnt].lineSeg1.g = g3;   pbuf->slc[cnt].lineSeg1.b = b3; }
					dt = dt + 1;
				}

				pbuf->slc[cnt].line_m = 1;
				if (dt < 2)  	flag = 1;
				if (dt > 2)   flag2 = 1;
			}

			//计算切片点位置
			if (flagc == 1) 
			{
				x1 = pbuf->slc[cnt].lineSeg0.x;   y1 = pbuf->slc[cnt].lineSeg0.y;
				x2 = pbuf->slc[cnt].lineSeg1.x;   y2 = pbuf->slc[cnt].lineSeg1.y;
				x3 = x2 - x1;  y3 = y2 - y1;
				if (xn * y3 - yn * x3 < 0)  pbuf->slc[cnt].line_n = -1;
				else  pbuf->slc[cnt].line_n = 1;
			}

		}
	}

	//删除buf.line_m[cnt]=2;重复线段（都在上侧，两段线都删除）
	//切线的两三角形在切平面的：1上下，2同侧，3一上一平，4两平    //上下选z>z9的三角形 
	for (k = 1; k <= cnt - 1; k++) 
	{
		if (pbuf->slc[k].line_m == 2) 
		{
			flag = 0;
			x1 = pbuf->slc[k].lineSeg0.x;   y1 = pbuf->slc[k].lineSeg0.y;
			x2 = pbuf->slc[k].lineSeg1.x;   y2 = pbuf->slc[k].lineSeg1.y;

			for (i = k + 1; i <= cnt; i++) 
			{
				x3 = pbuf->slc[i].lineSeg0.x;   y3 = pbuf->slc[i].lineSeg0.y;
				x4 = pbuf->slc[i].lineSeg1.x;   y4 = pbuf->slc[i].lineSeg1.y;
				if ((x1 == x3 && x2 == x4 && y1 == y3 && y2 == y4) 
					|| (x2 == x3 && x1 == x4 && y2 == y3 && y1 == y4)) 
				{
					flag = 1;
					//删除相同的
					for (j = i + 1; j <= cnt; j++) 
					{
						pbuf->slc[j - 1].lineSeg0 = pbuf->slc[j].lineSeg0;
						pbuf->slc[j - 1].lineSeg1 = pbuf->slc[j].lineSeg1;

						pbuf->slc[j - 1].line_n = pbuf->slc[j].line_n;
						pbuf->slc[j - 1].line_m = pbuf->slc[j].line_m;
					}
					cnt = cnt - 1;
				}
			}
			//同侧的两个线段都删除
			if (flag == 1) 
			{
				for (j = k + 1; j <= cnt; j++) 
				{
					pbuf->slc[j - 1].lineSeg0 = pbuf->slc[j].lineSeg0;
					pbuf->slc[j - 1].lineSeg1 = pbuf->slc[j].lineSeg1;

					pbuf->slc[j - 1].line_n = pbuf->slc[j].line_n;
					pbuf->slc[j - 1].line_m = pbuf->slc[j].line_m;
				}
				cnt = cnt - 1;
			}
		}
	}

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

	//输入STL格式
	input_stl_PNT("test1.stl", &buf);

	//旋转点云
	RotatePnt(&buf);

	//切片位置
	double z = 6;

	//切片结果到buf
	compute_slc(z, &buf);

	//旋转切片结果
	RotateSlcPnt(z, &buf);

	//输出点云
	output_slc_pmt(&buf, "E:\\test3.asc");

	//释放内存
	free(buf.PNT);
}


