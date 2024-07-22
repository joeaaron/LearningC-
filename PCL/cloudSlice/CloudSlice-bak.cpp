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

#undef min
#undef max

#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>

using namespace Eigen;

#define PNT_MAX  60000
#define ROT_MATRIX_SIZE 3
#define TRANSLATION_SIZE 3
#define PNT_STRIDE 12

// �������һ���ṹ�嶨�����£����ڴ洢�߶ε�����
typedef struct LineSegment 
{
	double x, y, z;					// �߶ζ˵������
	double r = 0, g = 0, b = 0;		// �߶ζ˵����ɫ

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
	float* PNT;    //��ŵ���
	long int ct;   //ԭʼ���Ƽ���
	int cnt;	   //��Ƭ���Ƽ���

	struct {
		LineSegment lineSeg0;
		LineSegment lineSeg1;
		int line_m, line_n;
		float x0, y0, z0;		// ����˵�0
		float x1, y1, z1;		// ����˵�1
	}slc[PNT_MAX];  //��Ƭ����
} buf;

// ����inVar��ĳ�ֽṹ�壬��������ת�ǶȺ�ƽ��ֵ
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
				p[j] = PNT[kk * PNT_STRIDE + i * 3 + j];
			}

			// ������תƽ�ƺ������ p2 = R5 * p1 + T5
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

// �������ڼ����߶εĶ˵��������ɫ
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

// �����߶ξ�����ͬ�������˵�
bool IsSegmentEqual(const LineSegment& lineSeg1, const LineSegment& lineSeg2,
	const LineSegment& slcLineSeg0, const LineSegment& slcLineSeg1)
{
	return (lineSeg1.x == slcLineSeg0.x && lineSeg2.x == slcLineSeg1.x && lineSeg1.y == slcLineSeg0.y && lineSeg2.y == slcLineSeg1.y)
		|| (lineSeg2.x == slcLineSeg0.x && lineSeg1.x == slcLineSeg1.x && lineSeg2.y == slcLineSeg0.y && lineSeg1.y == slcLineSeg1.y);
}
//******************************************************************
/**
 * @brief ������Ƭ
 *
 * ���ݸ����� z ֵ��z9���ͻ�������pbuf���е����ݣ�������Ƭ���洢�� pbuf �� slc �����С�
 *
 * @param z9 ��Ƭ�� z ֵ
 * @param pbuf �������ṹ��ָ�룬�����������������Ϣ
 */
void ComputeSlc(double z9, struct type_buf* pbuf)
{
	auto PNT = pbuf->PNT;
	int ct = pbuf->ct;
	int cnt = 0;

	LineSegment lineSeg1;
	LineSegment lineSeg2;
	LineSegment lineSeg3;

	//������Ƭ
	for (int k = 1; k <= ct && cnt + 1 < PNT_MAX; k++)
	{
		// ��ȡ��ǰ����������������
		double xn = pbuf->PNT[k * PNT_STRIDE + 1];
		double yn = pbuf->PNT[k * PNT_STRIDE + 2];
		double zn = pbuf->PNT[k * PNT_STRIDE + 3];

		lineSeg1.x = PNT[k * PNT_STRIDE + 4]; lineSeg1.y = PNT[k * PNT_STRIDE + 5]; lineSeg1.z = PNT[k * PNT_STRIDE + 6];
		lineSeg2.x = PNT[k * PNT_STRIDE + 7]; lineSeg2.y = PNT[k * PNT_STRIDE + 8]; lineSeg2.z = PNT[k * PNT_STRIDE + 9];
		lineSeg3.x = PNT[k * PNT_STRIDE + 10]; lineSeg3.y = PNT[k * PNT_STRIDE + 11]; lineSeg3.z = PNT[k * PNT_STRIDE + PNT_STRIDE];

		//����λ�����ٽ��ĵ�����
		if (lineSeg1.z == z9 && lineSeg2.z == z9 && lineSeg3.z == z9) { continue; }
		bool flagRelated = false;
		int dt = 0;

		// ����������ڵ㶼��z9ƽ���ͬ�ࣨ�������ڻ�С��z9�������һ������Ƿ���һ������������z9ƽ���ϡ�
		// ����ǣ�����Щ�߶δ洢��pbuf->slc�С�
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
		// ����������ڵ㲻��z9ƽ���ͬ�࣬�������z9ƽ���ཻ���߶Σ�������洢��pbuf->slc�С�
		else 
		{  
			//�Ե���бȶԺ��ɾ��
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

		//������Ƭ��λ�ã�����ÿ����z9ƽ���ཻ���߶Σ�ȷ��������Ƭƽ���ϵķ��򣨻��ڵ�ǰ����߶εķ��򣩡�
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

	//ɾ��buf.line_m[cnt]=2;�ظ��߶Σ������ϲ࣬�����߶�ɾ����
	//���ߵ�������������ƽ��ģ�1���£�2ͬ�࣬3һ��һƽ��4��ƽ    
	//����ѡz>z9�������� 
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
						pbuf->slc[j] = pbuf->slc[j + 1]; // ��ǰ�ƶ��߶�
					}
					--cnt; // ���ټ���
				}
			}
			//ͬ��������߶ζ�ɾ��
			if (flagSameSide)
			{
				for (int j = k; j <= cnt - 1; j++)
				{
					pbuf->slc[j] = pbuf->slc[j + 1]; // ��ǰ�ƶ��߶�
				}
				--cnt;
			}
		}
	}

	// ����pbuf->cnt�Է�ӳ�洢��pbuf->slc�е��߶�����
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

	// ����һ��pcl::PolygonMesh����
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

	// ����STL�ļ�
	if (pcl::io::loadPolygonFileSTL("test1.stl", *mesh) == -1) {
		return ;
	}

	//����STL��ʽ
	input_stl_PNT("test1.stl", &buf);

	//��ת����
	RotatePnt(&buf);

	//��Ƭλ��
	double z = 6;

	//��Ƭ�����buf
	ComputeSlc(z, &buf);

	//��ת��Ƭ���
	RotateSlcPnt(z, &buf);

	//�������
	output_slc_pmt(&buf, "E:\\test3.asc");

	//�ͷ��ڴ�
	//free(buf.PNT);
}


