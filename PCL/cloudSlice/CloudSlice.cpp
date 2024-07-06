#pragma warning(disable:4996)

#include "windows.h"
#include "math.h"  //数学函数库
#include "stdio.h"  //输入输出库
#include "string.h"  //字符库
#include "stdlib.h"  //流函数库

#define PNT_MAX  60000

struct type_in_var {
	float move_x, move_y, move_z;  //平移
	float rot_x, rot_y, rot_z;   //旋转
	double record_R5[4][4], record_T5[4];
} in_var;

struct type_buf {
	float* PNT;    //存放点云
	long int ct;   //点云计数
	int cnt;

	struct {
		double line_x0, line_y0, line_r0, line_g0, line_b0;
		double line_x1, line_y1, line_r1, line_g1, line_b1;
		int line_m, line_n;
		float x0, y0, z0, x1, y1, z1;
	}slc[PNT_MAX];  //切片点云
} buf;

void get_var(void);
void input_stl_PNT(const char*, struct type_buf*);
void rotate_stl_PNT(struct type_buf* pbuf);
void compute_slc(double, struct type_buf*);
void rotate_slc_pmt(double, struct type_buf* pbuf);
void output_slc_pmt(struct type_buf*, const char* name);

//***************************************************************
//输入参数
void get_var(void)
{
	//平移量
	in_var.move_x = 0;
	in_var.move_y = 0;
	in_var.move_z = 0;
	//旋转量
	in_var.rot_x = 0;
	in_var.rot_y = 0;
	in_var.rot_z = 0;
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

//***************************************************************************
//坐标系旋转
void rotate_stl_PNT(struct type_buf* pbuf)
{
	int i, j, k, kk, ct;
	double x, p[4 + 1];
	double pi, sin1, cos1, R1[4][4], R2[4][4], R3[4][4], R4[4][4];
	double R5[4][4], T5[4];
	float* PNT;

	PNT = pbuf->PNT;
	ct = pbuf->ct;

	//摆放旋转平移
	pi = 3.141592653589793;

	sin1 = sin(in_var.rot_x * pi / 180);  cos1 = cos(in_var.rot_x * pi / 180);
	R1[1][1] = 1;    R1[1][2] = 0;    R1[1][3] = 0;
	R1[2][1] = 0;    R1[2][2] = cos1;    R1[2][3] = -sin1;
	R1[3][1] = 0;    R1[3][2] = sin1;    R1[3][3] = cos1;

	sin1 = sin(in_var.rot_y * pi / 180);  cos1 = cos(in_var.rot_y * pi / 180);
	R2[1][1] = cos1;    R2[1][2] = 0;    R2[1][3] = sin1;
	R2[2][1] = 0;    R2[2][2] = 1;    R2[2][3] = 0;
	R2[3][1] = -sin1;    R2[3][2] = 0;    R2[3][3] = cos1;

	sin1 = sin(in_var.rot_z * pi / 180);  cos1 = cos(in_var.rot_z * pi / 180);
	R3[1][1] = cos1;    R3[1][2] = -sin1;    R3[1][3] = 0;
	R3[2][1] = sin1;    R3[2][2] = cos1;    R3[2][3] = 0;
	R3[3][1] = 0;    R3[3][2] = 0;    R3[3][3] = 1;

	for (i = 1; i <= 3; i++) {
		for (j = 1; j <= 3; j++) {
			x = 0;
			for (k = 1; k <= 3; k++) {
				x = x + R2[i][k] * R1[k][j];
			}
			R4[i][j] = x;
		}
	}

	for (i = 1; i <= 3; i++) {
		for (j = 1; j <= 3; j++) {
			x = 0;
			for (k = 1; k <= 3; k++) {
				x = x + R3[i][k] * R4[k][j];
			}
			R5[i][j] = x;
		}
	}

	//平移
	T5[1] = in_var.move_x;
	T5[2] = in_var.move_y;
	T5[3] = in_var.move_z;

	//旋转
	for (kk = 1; kk <= ct; kk++) {
		for (i = 0; i <= 3; i++) {
			for (j = 1; j <= 3; j++) {
				if (i == 0)  p[j] = PNT[kk * 12 + i * 3 + j];
				if (i >= 1)  p[j] = PNT[kk * 12 + i * 3 + j];     //旋转中心在坐标系原点
			}

			//点云旋转平移后的坐标 p2=R5*p1+T
			for (j = 1; j <= 3; j++) {
				x = 0;
				for (k = 1; k <= 3; k++) {
					x = x + R5[j][k] * p[k];
				}
				if (i == 0)  PNT[kk * 12 + i * 3 + j] = x;
				if (i >= 1)  PNT[kk * 12 + i * 3 + j] = x + T5[j];
			}
		}
	}

	for (i = 1; i <= 3; i++) {
		for (j = 1; j <= 3; j++) {
			in_var.record_R5[i][j] = R5[i][j];
		}
		in_var.record_T5[i] = T5[i];
	}
}

//******************************************************************
//计算切片
void compute_slc(double z9, struct type_buf* pbuf)
{
	int i, j, k, ct, cnt, dt, flag = 0, flag2 = 0, flagc;
	double x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, xn, yn, zn, x9, y9;
	float* PNT;
	double  r, g, b, r1, g1, b1, r2, g2, b2, r3, g3, b3;

	PNT = pbuf->PNT;
	ct = pbuf->ct;
	cnt = 0;
	//计算切片
	for (k = 1; k <= ct; k++) {
		if (cnt + 1 < PNT_MAX) {

			xn = PNT[k * 12 + 1]; yn = PNT[k * 12 + 2]; zn = PNT[k * 12 + 3];
			x1 = PNT[k * 12 + 4]; y1 = PNT[k * 12 + 5]; z1 = PNT[k * 12 + 6];
			x2 = PNT[k * 12 + 7]; y2 = PNT[k * 12 + 8]; z2 = PNT[k * 12 + 9];
			x3 = PNT[k * 12 + 10]; y3 = PNT[k * 12 + 11]; z3 = PNT[k * 12 + 12];

			r1 = 0; g1 = 0; b1 = 0; r2 = 0; g2 = 0; b2 = 0; r3 = 0; g3 = 0; b3 = 0;
			//查找位置相临近的点坐标
			flagc = 0;
			if ((z1 >= z9 && z2 >= z9 && z3 >= z9) || (z1 <= z9 && z2 <= z9 && z3 <= z9)) {
				if (z1 == z9 && z2 == z9 && z3 == z9) {}
				else {
					if (z1 == z9 && z2 == z9 && z3 > z9) {
						cnt = cnt + 1;  dt = 0;   flagc = 1;
						pbuf->slc[cnt].line_x0 = x1;   pbuf->slc[cnt].line_y0 = y1;
						pbuf->slc[cnt].line_r0 = r1;   pbuf->slc[cnt].line_g0 = g1;   pbuf->slc[cnt].line_b0 = b1;		dt = dt + 1;
						pbuf->slc[cnt].line_x1 = x2;   pbuf->slc[cnt].line_y1 = y2;
						pbuf->slc[cnt].line_r1 = r2;   pbuf->slc[cnt].line_g1 = g2;   pbuf->slc[cnt].line_b1 = b2;		dt = dt + 1;
						pbuf->slc[cnt].line_m = 2;
					}
					if (z1 == z9 && z3 == z9 && z2 > z9) {
						cnt = cnt + 1;  dt = 0;   flagc = 1;
						pbuf->slc[cnt].line_x0 = x1;   pbuf->slc[cnt].line_y0 = y1;
						pbuf->slc[cnt].line_r0 = r1;   pbuf->slc[cnt].line_g0 = g1;   pbuf->slc[cnt].line_b0 = b1;		dt = dt + 1;
						pbuf->slc[cnt].line_x1 = x3;   pbuf->slc[cnt].line_y1 = y3;
						pbuf->slc[cnt].line_r1 = r3;   pbuf->slc[cnt].line_g1 = g3;   pbuf->slc[cnt].line_b1 = b3;		dt = dt + 1;
						pbuf->slc[cnt].line_m = 2;
					}
					if (z3 == z9 && z2 == z9 && z1 > z9) {
						cnt = cnt + 1;  dt = 0;   flagc = 1;
						pbuf->slc[cnt].line_x0 = x3;   pbuf->slc[cnt].line_y0 = y3;
						pbuf->slc[cnt].line_r0 = r3;   pbuf->slc[cnt].line_g0 = g3;   pbuf->slc[cnt].line_b0 = b3;		dt = dt + 1;
						pbuf->slc[cnt].line_x1 = x2;   pbuf->slc[cnt].line_y1 = y2;
						pbuf->slc[cnt].line_r1 = r2;   pbuf->slc[cnt].line_g1 = g2;   pbuf->slc[cnt].line_b1 = b2;		dt = dt + 1;
						pbuf->slc[cnt].line_m = 2;
					}
				}//else

			}
			else {  //对点进行比对后的删除
				cnt = cnt + 1;  dt = 0;   flagc = 1;

				if ((z1 - z9) * (z2 - z9) < 0) {
					x9 = (z9 - z1) * (x2 - x1) / (z2 - z1) + x1;
					y9 = (z9 - z1) * (y2 - y1) / (z2 - z1) + y1;
					if (dt == 0) { pbuf->slc[cnt].line_x0 = x9;   pbuf->slc[cnt].line_y0 = y9; }
					if (dt == 1) { pbuf->slc[cnt].line_x1 = x9;   pbuf->slc[cnt].line_y1 = y9; }
					//
					r = (z9 - z1) * (r2 - r1) / (z2 - z1) + r1;
					g = (z9 - z1) * (g2 - g1) / (z2 - z1) + g1;
					b = (z9 - z1) * (b2 - b1) / (z2 - z1) + b1;
					if (dt == 0) { pbuf->slc[cnt].line_r0 = r;   pbuf->slc[cnt].line_g0 = g;   pbuf->slc[cnt].line_b0 = b; }
					if (dt == 1) { pbuf->slc[cnt].line_r1 = r;   pbuf->slc[cnt].line_g1 = g;   pbuf->slc[cnt].line_b1 = b; }
					dt = dt + 1;
				}
				if ((z1 - z9) * (z3 - z9) < 0) {
					x9 = (z9 - z1) * (x3 - x1) / (z3 - z1) + x1;
					y9 = (z9 - z1) * (y3 - y1) / (z3 - z1) + y1;
					if (dt == 0) { pbuf->slc[cnt].line_x0 = x9;   pbuf->slc[cnt].line_y0 = y9; }
					if (dt == 1) { pbuf->slc[cnt].line_x1 = x9;   pbuf->slc[cnt].line_y1 = y9; }
					//
					r = (z9 - z1) * (r3 - r1) / (z3 - z1) + r1;
					g = (z9 - z1) * (g3 - g1) / (z3 - z1) + g1;
					b = (z9 - z1) * (b3 - b1) / (z3 - z1) + b1;
					if (dt == 0) { pbuf->slc[cnt].line_r0 = r;   pbuf->slc[cnt].line_g0 = g;   pbuf->slc[cnt].line_b0 = b; }
					if (dt == 1) { pbuf->slc[cnt].line_r1 = r;   pbuf->slc[cnt].line_g1 = g;   pbuf->slc[cnt].line_b1 = b; }
					dt = dt + 1;
				}
				if ((z3 - z9) * (z2 - z9) < 0) {
					x9 = (z9 - z3) * (x2 - x3) / (z2 - z3) + x3;
					y9 = (z9 - z3) * (y2 - y3) / (z2 - z3) + y3;
					if (dt == 0) { pbuf->slc[cnt].line_x0 = x9;   pbuf->slc[cnt].line_y0 = y9; }
					if (dt == 1) { pbuf->slc[cnt].line_x1 = x9;   pbuf->slc[cnt].line_y1 = y9; }
					//
					r = (z9 - z3) * (r2 - r3) / (z2 - z3) + r3;
					g = (z9 - z3) * (g2 - g3) / (z2 - z3) + g3;
					b = (z9 - z3) * (b2 - b3) / (z2 - z3) + b3;
					if (dt == 0) { pbuf->slc[cnt].line_r0 = r;   pbuf->slc[cnt].line_g0 = g;   pbuf->slc[cnt].line_b0 = b; }
					if (dt == 1) { pbuf->slc[cnt].line_r1 = r;   pbuf->slc[cnt].line_g1 = g;   pbuf->slc[cnt].line_b1 = b; }
					dt = dt + 1;
				}
				if (z1 == z9 && (z3 - z9) * (z2 - z9) < 0) {
					if (dt == 0) { pbuf->slc[cnt].line_x0 = x1;   pbuf->slc[cnt].line_y0 = y1; }
					if (dt == 1) { pbuf->slc[cnt].line_x1 = x1;   pbuf->slc[cnt].line_y1 = y1; }
					if (dt == 0) { pbuf->slc[cnt].line_r0 = r1;   pbuf->slc[cnt].line_g0 = g1;   pbuf->slc[cnt].line_b0 = b1; }
					if (dt == 1) { pbuf->slc[cnt].line_r1 = r1;   pbuf->slc[cnt].line_g1 = g1;   pbuf->slc[cnt].line_b1 = b1; }
					dt = dt + 1;
				}
				if (z2 == z9 && (z1 - z9) * (z3 - z9) < 0) {
					if (dt == 0) { pbuf->slc[cnt].line_x0 = x2;   pbuf->slc[cnt].line_y0 = y2; }
					if (dt == 1) { pbuf->slc[cnt].line_x1 = x2;   pbuf->slc[cnt].line_y1 = y2; }
					if (dt == 0) { pbuf->slc[cnt].line_r0 = r2;   pbuf->slc[cnt].line_g0 = g2;   pbuf->slc[cnt].line_b0 = b2; }
					if (dt == 1) { pbuf->slc[cnt].line_r1 = r2;   pbuf->slc[cnt].line_g1 = g2;   pbuf->slc[cnt].line_b1 = b2; }
					dt = dt + 1;
				}
				if (z3 == z9 && (z1 - z9) * (z2 - z9) < 0) {
					if (dt == 0) { pbuf->slc[cnt].line_x0 = x3;   pbuf->slc[cnt].line_y0 = y3; }
					if (dt == 1) { pbuf->slc[cnt].line_x1 = x3;   pbuf->slc[cnt].line_y1 = y3; }
					if (dt == 0) { pbuf->slc[cnt].line_r0 = r3;   pbuf->slc[cnt].line_g0 = g3;   pbuf->slc[cnt].line_b0 = b3; }
					if (dt == 1) { pbuf->slc[cnt].line_r1 = r3;   pbuf->slc[cnt].line_g1 = g3;   pbuf->slc[cnt].line_b1 = b3; }
					dt = dt + 1;
				}

				pbuf->slc[cnt].line_m = 1;
				if (dt < 2)  	flag = 1;
				if (dt > 2)   flag2 = 1;
			}
			//计算切片点位置
			if (flagc == 1) {
				x1 = pbuf->slc[cnt].line_x0;   y1 = pbuf->slc[cnt].line_y0;
				x2 = pbuf->slc[cnt].line_x1;   y2 = pbuf->slc[cnt].line_y1;
				x3 = x2 - x1;  y3 = y2 - y1;
				if (xn * y3 - yn * x3 < 0)  pbuf->slc[cnt].line_n = -1;
				else  pbuf->slc[cnt].line_n = 1;
			}

		}
	}

	//删除buf.line_m[cnt]=2;重复线段（都在上侧，两段线都删除）
	//切线的两三角形在切平面的：1上下，2同侧，3一上一平，4两平    //上下选z>z9的三角形 
	for (k = 1; k <= cnt - 1; k++) {
		if (pbuf->slc[k].line_m == 2) {
			flag = 0;
			x1 = pbuf->slc[k].line_x0;   y1 = pbuf->slc[k].line_y0;
			x2 = pbuf->slc[k].line_x1;   y2 = pbuf->slc[k].line_y1;
			for (i = k + 1; i <= cnt; i++) {
				x3 = pbuf->slc[i].line_x0;   y3 = pbuf->slc[i].line_y0;
				x4 = pbuf->slc[i].line_x1;   y4 = pbuf->slc[i].line_y1;
				if ((x1 == x3 && x2 == x4 && y1 == y3 && y2 == y4) || (x2 == x3 && x1 == x4 && y2 == y3 && y1 == y4)) {
					flag = 1;
					//删除相同的
					for (j = i + 1; j <= cnt; j++) {
						pbuf->slc[j - 1].line_x0 = pbuf->slc[j].line_x0;
						pbuf->slc[j - 1].line_y0 = pbuf->slc[j].line_y0;
						pbuf->slc[j - 1].line_x1 = pbuf->slc[j].line_x1;
						pbuf->slc[j - 1].line_y1 = pbuf->slc[j].line_y1;

						pbuf->slc[j - 1].line_r0 = pbuf->slc[j].line_r0;
						pbuf->slc[j - 1].line_g0 = pbuf->slc[j].line_g0;
						pbuf->slc[j - 1].line_b0 = pbuf->slc[j].line_b0;
						pbuf->slc[j - 1].line_r1 = pbuf->slc[j].line_r1;
						pbuf->slc[j - 1].line_g1 = pbuf->slc[j].line_g1;
						pbuf->slc[j - 1].line_b1 = pbuf->slc[j].line_b1;

						pbuf->slc[j - 1].line_n = pbuf->slc[j].line_n;
						pbuf->slc[j - 1].line_m = pbuf->slc[j].line_m;
					}
					cnt = cnt - 1;
				}
			}
			//同侧的两个线段都删除
			if (flag == 1) {
				for (j = k + 1; j <= cnt; j++) {
					pbuf->slc[j - 1].line_x0 = pbuf->slc[j].line_x0;
					pbuf->slc[j - 1].line_y0 = pbuf->slc[j].line_y0;
					pbuf->slc[j - 1].line_x1 = pbuf->slc[j].line_x1;
					pbuf->slc[j - 1].line_y1 = pbuf->slc[j].line_y1;

					pbuf->slc[j - 1].line_r0 = pbuf->slc[j].line_r0;
					pbuf->slc[j - 1].line_g0 = pbuf->slc[j].line_g0;
					pbuf->slc[j - 1].line_b0 = pbuf->slc[j].line_b0;
					pbuf->slc[j - 1].line_r1 = pbuf->slc[j].line_r1;
					pbuf->slc[j - 1].line_g1 = pbuf->slc[j].line_g1;
					pbuf->slc[j - 1].line_b1 = pbuf->slc[j].line_b1;

					pbuf->slc[j - 1].line_n = pbuf->slc[j].line_n;
					pbuf->slc[j - 1].line_m = pbuf->slc[j].line_m;
				}
				cnt = cnt - 1;
			}
		}
	}

	pbuf->cnt = cnt;
}

//************************************************************
//旋转切片点云
void rotate_slc_pmt(double z, struct type_buf* pbuf)
{
	int i, j, k;
	double x, p[4 + 1], q[4 + 1];
	double R5[4][4], T5[4];

	//旋转矩阵
	for (i = 1; i <= 3; i++) {
		for (j = 1; j <= 3; j++) {
			R5[i][j] = in_var.record_R5[i][j];
		}
		T5[i] = in_var.record_T5[i];
	}

	//旋转切片端点0
	for (k = 1; k <= pbuf->cnt; k++) {
		p[1] = pbuf->slc[k].line_x0 - T5[1];
		p[2] = pbuf->slc[k].line_y0 - T5[2];
		p[3] = z - T5[3];
		//点云旋转平移后的坐标 p2=R5*p1+T,  逆变换p1=R5'*(p2-T)
		for (i = 1; i <= 3; i++) {
			x = 0;
			for (j = 1; j <= 3; j++) {
				x = x + R5[j][i] * p[j];
			}
			q[i] = x;
		}
		pbuf->slc[k].x0 = q[1];
		pbuf->slc[k].y0 = q[2];
		pbuf->slc[k].z0 = q[3];
	}

	//旋转切片端点1
	for (k = 1; k <= pbuf->cnt; k++) {
		p[1] = pbuf->slc[k].line_x1 - T5[1];
		p[2] = pbuf->slc[k].line_y1 - T5[2];
		p[3] = z - T5[3];
		//点云旋转平移后的坐标 p2=R5*p1+T,  逆变换p1=R5'*(p2-T)
		for (i = 1; i <= 3; i++) {
			x = 0;
			for (j = 1; j <= 3; j++) {
				x = x + R5[j][i] * p[j];
			}
			q[i] = x;
		}
		pbuf->slc[k].x1 = q[1];
		pbuf->slc[k].y1 = q[2];
		pbuf->slc[k].z1 = q[3];
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
	rotate_stl_PNT(&buf);

	//切片位置
	double z = 6;

	//切片结果到buf
	compute_slc(z, &buf);

	//旋转切片结果
	rotate_slc_pmt(z, &buf);

	//输出点云
	output_slc_pmt(&buf, "E:\\test1.asc");

	//释放内存
	free(buf.PNT);
}


