#pragma warning(disable:4996)

#include "windows.h"
#include "math.h"  //��ѧ������
#include "stdio.h"  //���������
#include "string.h"  //�ַ���
#include "stdlib.h"  //��������

#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <iostream>
#include <cmath>

using namespace Eigen;

#define PNT_MAX  60000
#define ROT_MATRIX_SIZE 3
#define TRANSLATION_SIZE 3

// �������һ���ṹ�嶨�����£����ڴ洢�߶ε�����
typedef struct {
	double x, y;		// �߶ζ˵������
	double r, g, b;		// �߶ζ˵����ɫ
} LineSegment;

struct type_buf 
{
	float* PNT;    //��ŵ���
	long int ct;   //���Ƽ���
	int cnt;

	struct {
		LineSegment lineSeg0;
		LineSegment lineSeg1;
		int line_m, line_n;
		float x0, y0, z0;
		float x1, y1, z1;
	}slc[PNT_MAX];  //��Ƭ����
} buf;

// ����in_var��ĳ�ֽṹ�壬��������ת�ǶȺ�ƽ��ֵ
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
//�������
void get_var(void)
{
	//ƽ����
	inVar.move_x = 0;
	inVar.move_y = 0;
	inVar.move_z = 0;
	//��ת��
	inVar.rot_x = 0;
	inVar.rot_y = 0;
	inVar.rot_z = 0;
}

//***************************************************************************
//�������
void input_stl_PNT(const char* name, struct type_buf* pbuf)
{
	short int PNT2;
	float PNT1[80 + 1];
	int i, j, k;
	long int ct;
	FILE* fid;

	if ((fid = fopen(name, "rb")) == NULL) {
		printf("ģ���ļ��򿪴���");   return;
	}
	for (i = 1; i <= 20; i++)  PNT1[i] = 0;
	fread(&PNT1[1], sizeof(float), 20, fid);
	fread(&ct, sizeof(long int), 1, fid);
	PNT2 = 0;

	//�����ڴ�
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

	// �ڷ���תƽ��
	double sin_x = sin(inVar.rot_x * M_PI / 180);
	double cos_x = cos(inVar.rot_x * M_PI / 180);

	// ������ת���� R1, R2, R3
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

	// ���� R4 = R2 * R1, R5 = R3 * R4
	Matrix3d R4 = R2 * R1;
	Matrix3d R5 = R3 * R4;

	// ƽ������
	Vector3d T5(inVar.move_x, inVar.move_y, inVar.move_z);

	// ��ת��ƽ�Ƶ���
	for (int kk = 0; kk < ct; kk++) 
	{
		for (int i = 0; i < 4; i++) 
		{
			Vector3d p;
			for (int j = 0; j < 3; j++) 
			{
				p[j] = PNT[kk * 12 + i * 3 + j];
			}

			// ������תƽ�ƺ������ p2 = R5 * p1 + T5
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

	// ��¼��ת�����ƽ������
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
//������Ƭ
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

	//������Ƭ
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

			//����λ�����ٽ��ĵ�����
			flagc = 0;
			if ((z1 >= z9 && z2 >= z9 && z3 >= z9) || (z1 <= z9 && z2 <= z9 && z3 <= z9)) 
			{
				if (z1 == z9 && z2 == z9 && z3 == z9) {}
				else 
				{
					if (z1 == z9 && z2 == z9 && z3 > z9)
					{
						cnt = cnt + 1;  dt = 0;   flagc = 1;

						// �����߶εĶ˵��������ɫ
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
			{  //�Ե���бȶԺ��ɾ��
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

			//������Ƭ��λ��
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

	//ɾ��buf.line_m[cnt]=2;�ظ��߶Σ������ϲ࣬�����߶�ɾ����
	//���ߵ�������������ƽ��ģ�1���£�2ͬ�࣬3һ��һƽ��4��ƽ    //����ѡz>z9�������� 
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
					//ɾ����ͬ��
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
			//ͬ��������߶ζ�ɾ��
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
 * @brief ��ת SLC ��Ƭ��
 *
 * ���ݸ�������ת�����ƽ���������� SLC ��Ƭ�еĵ������ת��ƽ�Ʋ�����
 *
 * @param z ��תƽ��� Z ����
 * @param pbuf SLC ��Ƭ���ݵ�ָ��
 */
void RotateSlcPnt(double z, struct type_buf* pbuf)
{
	Eigen::Matrix3d R;
	Eigen::Vector3d T;

	// ��ת����
	for (int i = 0; i < ROT_MATRIX_SIZE; i++) 
	{
		for (int j = 0; j < ROT_MATRIX_SIZE; j++) 
		{
			R(i, j) = inVar.mtxR(i + 1, j + 1); 
		}
		T(i) = inVar.vecT(i + 1); 
	}

	// �����������Ը����ĵ������ת��ƽ��
	auto rotateAndTranslate = [&](double x, double y, double& newX, double& newY, double& newZ) {
		Eigen::Vector3d p(x, y, z);
		Eigen::Vector3d q = R * (p - T) + T;
		newX = q.x();
		newY = q.y();
		newZ = q.z();
		};

	// ����Ƭ�˵�0��1������ת��ƽ��
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
//�������
void output_slc_pmt(struct type_buf* pbuf, const char* name)
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
	for (k = 1; k <= pbuf->cnt; k++) {
		x1 = pbuf->slc[k].x0;
		y1 = pbuf->slc[k].y0;
		z1 = pbuf->slc[k].z0;
		x2 = pbuf->slc[k].x1;
		y2 = pbuf->slc[k].y1;
		z2 = pbuf->slc[k].z1;
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

//***************************************************************************
//������
void main(void)
{
	//�������
	get_var();

	//����STL��ʽ
	input_stl_PNT("test1.stl", &buf);

	//��ת����
	RotatePnt(&buf);

	//��Ƭλ��
	double z = 6;

	//��Ƭ�����buf
	compute_slc(z, &buf);

	//��ת��Ƭ���
	RotateSlcPnt(z, &buf);

	//�������
	output_slc_pmt(&buf, "E:\\test3.asc");

	//�ͷ��ڴ�
	free(buf.PNT);
}


