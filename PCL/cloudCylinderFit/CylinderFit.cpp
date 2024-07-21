#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <time.h>
#include <chrono>
#include <fstream>

using namespace std;

using Point = Eigen::Vector3d;

void ReadData(const char* path, std::vector<Point>& pts)
{
	FILE* fp = fopen(path, "r+");
	if (!fp) return;
	pts.clear();
	double x, y, z, nx, ny, nz;
	while (!feof(fp))
	{
		fscanf(fp, "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &nx, &ny, &nz);
		pts.push_back(Point(x, y, z));
	}
	fclose(fp);
}

void ReadData3(const char* path, std::vector<Point>& pts)
{
	FILE* fp = fopen(path, "r+");
	if (!fp) return;
	pts.clear();
	double x, y, z;
	while (!feof(fp))
	{
		fscanf(fp, "%lf %lf %lf", &x, &y, &z);
		pts.push_back(Point(x, y, z));
	}
	fclose(fp);
}

std::vector<Point> ReadPointsFromFile(const std::string& filename)
{
	std::vector<Point> points;
	std::ifstream file(filename);

	if (!file.is_open()) {
		std::cerr << "Could not open the file!" << std::endl;
		return points;
	}

	std::string line;
	while (std::getline(file, line))
	{
		std::replace(line.begin(), line.end(), ',', ' ');
		std::istringstream iss(line);
		double x, y, z;
		if (!(iss >> x >> y >> z))
		{
			std::cerr << "Error parsing line: " << line << std::endl;
			break;
		}
		points.emplace_back(Point(x, y, z));
	}

	file.close();
	return points;
}

void WriteData3(const char* path, const std::vector<Point>& pts)
{
	FILE* fp = fopen(path, "w+");
	if (!fp) return;

	double x, y, z;
	int i = 0;
	while (i < pts.size())
	{
		x = pts[i].x();
		y = pts[i].y();
		z = pts[i].z();
		fprintf(fp, "%lf %lf %lf\n", x, y, z);
		i++;
	}
	fclose(fp);
}

//向量转反对称矩阵
Eigen::Matrix3d Hat(const Eigen::Vector3d& v)
{
	Eigen::Matrix3d Omega;
	Omega << 0, -v(2), v(1)
		, v(2), 0, -v(0)
		, -v(1), v(0), 0;
	return Omega;
}

//旋转向量转四元数
Eigen::Quaterniond Exp(const Eigen::Vector3d& omega)
{
	double theta = omega.norm();
	double half_theta = 0.5 * theta;

	double imag_factor;
	double real_factor = cos(half_theta);
	if (theta < 1.0e-10)
	{
		double theta_sq = theta * theta;
		double theta_po4 = theta_sq * theta_sq;
		imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4;
	}
	else
	{
		double sin_half_theta = sin(half_theta);
		imag_factor = sin_half_theta / theta;
	}

	return Eigen::Quaterniond(real_factor,
		imag_factor * omega.x(),
		imag_factor * omega.y(),
		imag_factor * omega.z());
}

//拟合圆柱
void FitCylinder(const std::vector<Eigen::Vector3d>& pts)
{
	auto start = chrono::steady_clock().now();
	//初始化
	Eigen::Vector3d center(0, 0, 0);
	for (size_t i = 0; i < pts.size(); i++)
	{
		center += pts[i];
	}
	center /= pts.size();

	Eigen::Vector3d normal;
	normal(0) = 0;
	normal(1) = 1;
	normal(2) = 0;
	normal.normalize();


	double r = 0.0;
	for (size_t i = 0; i < pts.size(); i++)
	{
		r += (pts[i] - center).cross(normal).norm();
	}
	r /= pts.size();

	printf("INIT radius: %lf\n", r);
	printf("INIT center: %lf %lf %lf\n", center(0), center(1), center(2));
	printf("INIT normal: %lf %lf %lf\n", normal(0), normal(1), normal(2));

	Eigen::Matrix<double, 7, 7> Hessian;
	Eigen::Matrix<double, 1, 7> g;
	Eigen::Vector<double, 7> Jacb;

	double pre_rms = 0.0;
	double last_rms = 1e10;
	const double stop_rms = 1e-7;

	int max_iters = 1000;
	while (max_iters-- > 0)
	{
		//初始化
		pre_rms = last_rms;
		Hessian.setZero();
		Jacb.setZero();

		double x0 = center(0);
		double y0 = center(1);
		double z0 = center(2);
		double a = normal(0);
		double b = normal(1);
		double c = normal(2);

		//计算雅可比向量与海森矩阵
		last_rms = 0.0;
		for (size_t i = 0; i < pts.size(); i++)
		{
			double x = pts[i].x();
			double y = pts[i].y();
			double z = pts[i].z();
			double dx = x - x0;
			double dy = y - y0;
			double dz = z - z0;
			double u = c * dy - b * dz;
			double v = a * dz - c * dx;
			double w = b * dx - a * dy;
			double di = sqrt(u * u + v * v + w * w);
			double M = di - r;

			//单个点的雅可比向量
			double error = 0.5 * (M * M);
			g(0, 0) = M * (dz * v - dy * w) / di;
			g(0, 1) = M * (dx * w - dz * u) / di;
			g(0, 2) = M * (dy * u - dx * v) / di;
			g(0, 3) = M * (c * v - b * w) / di;
			g(0, 4) = M * (a * w - c * u) / di;
			g(0, 5) = M * (b * u - a * v) / di;
			g(0, 6) = -M;

			//法向对R求导
			Eigen::Matrix3d RN = -Hat(normal);
			g.block<1, 3>(0, 0) = g.block<1, 3>(0, 0) * RN;

			Eigen::Vector<double, 7> tJ;
			for (size_t j = 0; j < 7; j++)
			{
				tJ(j) = g(0, j);
			}

			Hessian += tJ * tJ.transpose();
			Jacb += error * g;

			last_rms += error;
		}
		last_rms /= pts.size();

		printf("#ITERS:%d, RMSE:%lf\n", max_iters, last_rms);

		//参数增量
		Eigen::Vector<double, 7> X = Hessian.fullPivHouseholderQr().solve(-Jacb);

		//使用旋转矩阵R更新向量
		Eigen::Matrix3d R = Exp(Eigen::Vector3d(X(0), X(1), X(2))).matrix();
		normal = R * normal;

		//更新O点与底圆半径
		center(0) += X(3);
		center(1) += X(4);
		center(2) += X(5);
		r += X(6);

		//前后两次误差增量小于一定值，退出
		if (fabs(last_rms - pre_rms) < stop_rms) break;
	}

	printf("END radius: %lf\n", r);
	printf("END center: %lf %lf %lf\n", center(0), center(1), center(2));
	printf("END normal: %lf %lf %lf\n", normal(0), normal(1), normal(2));

	auto end = chrono::steady_clock().now();
	double cost_time = chrono::duration_cast<chrono::milliseconds>(end - start).count();
	printf("cost time: %lf ms\n", cost_time);
}

//拟合圆锥
void FitCone(const std::vector<Eigen::Vector3d>& pts)
{
	auto start = chrono::steady_clock().now();
	//初始化
	Eigen::Vector3d center(0, 0, 0);
	for (size_t i = 0; i < pts.size(); i++)
	{
		center += pts[i];
	}
	center /= pts.size();

	Eigen::Vector3d normal;
	normal(0) = 0;
	normal(1) = 1;
	normal(2) = 0;
	normal.normalize();


	double alph = 0.4;
	double r = 0.0;
	for (size_t i = 0; i < pts.size(); i++)
	{
		r += (pts[i] - center).cross(normal).norm();
	}
	r /= pts.size();

	printf("INIT radius: %lf\n", r);
	printf("INIT angle : %lf\n", alph);
	printf("INIT center: %lf %lf %lf\n", center(0), center(1), center(2));
	printf("INIT normal: %lf %lf %lf\n", normal(0), normal(1), normal(2));

	Eigen::Matrix<double, 8, 8> Hessian;
	Eigen::Matrix<double, 1, 8> g;
	Eigen::Vector<double, 8> Jacb;

	double pre_rms = 0.0;
	double last_rms = 1e10;
	const double stop_rms = 1e-7;

	int max_iters = 1000;
	while (max_iters-- > 0)
	{
		//初始化
		pre_rms = last_rms;
		Hessian.setZero();
		Jacb.setZero();

		double x0 = center(0);
		double y0 = center(1);
		double z0 = center(2);
		double a = normal(0);
		double b = normal(1);
		double c = normal(2);
		double cosa2 = cos(alph / 2);
		double sina2 = sin(alph / 2);
		//计算雅可比向量与海森矩阵
		last_rms = 0.0;
		for (size_t i = 0; i < pts.size(); i++)
		{
			double x = pts[i].x();
			double y = pts[i].y();
			double z = pts[i].z();
			double dx = x - x0;
			double dy = y - y0;
			double dz = z - z0;
			double u = c * dy - b * dz;
			double v = a * dz - c * dx;
			double w = b * dx - a * dy;
			double e = sqrt(u * u + v * v + w * w);
			double f = a * dx + b * dy + c * dz;
			double di = e * cosa2 + f * sina2 - r * cosa2;
			//单个点的雅可比向量
			double error = 0.5 * (di * di);
			g(0, 0) = di * (cosa2 * (dz * v - dy * w) / e + sina2 * dx);
			g(0, 1) = di * (cosa2 * (dx * w - dz * u) / e + sina2 * dy);
			g(0, 2) = di * (cosa2 * (dy * u - dx * v) / e + sina2 * dz);
			g(0, 3) = di * (cosa2 * (c * v - b * w) / e - sina2 * a);
			g(0, 4) = di * (cosa2 * (a * w - c * u) / e - sina2 * b);
			g(0, 5) = di * (cosa2 * (b * u - a * v) / e - sina2 * c);
			g(0, 6) = -cosa2 * di;
			g(0, 7) = di * (-0.5 * e * sina2 + 0.5 * f * cosa2 + 0.5 * r * sina2);

			//法向对R求导
			Eigen::Matrix3d RN = -Hat(normal);
			g.block<1, 3>(0, 0) = g.block<1, 3>(0, 0) * RN;

			Eigen::Vector<double, 8> tJ;
			for (size_t j = 0; j < 8; j++)
			{
				tJ(j) = g(0, j);
			}

			Hessian += tJ * tJ.transpose();
			Jacb += error * g;

			last_rms += error;
		}
		last_rms /= pts.size();

		printf("#ITERS:%d, RMSE:%lf\n", max_iters, last_rms);

		//参数增量
		Eigen::Vector<double, 8> X = Hessian.fullPivHouseholderQr().solve(-Jacb);

		//旋转向量转矩阵
		Eigen::Matrix3d R = Exp(Eigen::Vector3d(X(0), X(1), X(2))).matrix();
		//使用旋转矩阵R更新向量
		normal = R * normal;

		//更新O点与底圆半径
		center(0) += X(3);
		center(1) += X(4);
		center(2) += X(5);
		r += X(6);
		alph += X(7);
		//前后两次误差增量小于一定值，退出
		if (fabs(last_rms - pre_rms) < stop_rms) break;
	}

	printf("END radius: %lf\n", r);
	printf("END angle : %lf\n", cos(alph / 2));
	printf("END center: %lf %lf %lf\n", center(0), center(1), center(2));
	printf("END normal: %lf %lf %lf\n", normal(0), normal(1), normal(2));

	double d = r / tan(fabs(alph / 2));
	Eigen::Vector3d O = center - d * normal;
	printf("END orig  : %lf %lf %lf\n", O(0), O(1), O(2));

	auto end = chrono::steady_clock().now();
	double cost_time = chrono::duration_cast<chrono::milliseconds>(end - start).count();
	printf("cost time: %lf ms\n", cost_time);
}

//生成圆锥数据
void ProductConeData(std::vector<Point>& pts)
{
	int sum = 1000;
	pts.resize(sum);

	double cost = 0.8;
	printf("Product angle :%lf\n", (cost));

	Eigen::Vector3d n(0.7, 0.2, 0.5);
	n.normalize();
	printf("Product normal: %lf %lf %lf\n", n(0), n(1), n(2));

	Eigen::Vector3d O(12, 10, 20);
	printf("Product orig  : %lf %lf %lf\n", O(0), O(1), O(2));

	double d = 10;
	double noise = 0.03;

	int step = 10;
	int num = sum / step;
	double angle = 2 * 3.14159265358 / num;

	Eigen::Quaterniond q;
	q.setFromTwoVectors(Eigen::Vector3d(0, 0, 1), n);

	srand((unsigned int)time(NULL));
	for (int i = 1; i < 1 + step; i++)
	{
		double dd = d * i;
		double ld = dd / cost;
		double rd = sqrt(ld * ld - dd * dd);
		Eigen::Vector3d O1 = O + dd * n;

		for (size_t j = 0; j < num; j++)
		{
			double a = angle * j;
			double cosa = cos(a);
			double sina = sin(a);
			int noise_multi = rand() % 10;
			double r = rd + noise * noise_multi;

			Eigen::Vector3d p = Eigen::Vector3d(cosa, sina, 0.0) * r;
			Eigen::Vector3d p1 = O1 + q * p;
			pts[(i - 1) * num + j] = p1;
		}
	}
}


int main()
{
	std::vector<Point> pts;

	//ReadData("../data/cylinder1.txt", pts);
	//FitCylinder(pts);

	//ReadData3("../data/cylinder2.txt", pts);
	//FitCylinder(pts);

	//ReadData3("../data/cylinder3.txt", pts);
	//FitCylinder(pts);

	//ReadData3("../data/cylinder4.txt", pts);
	//FitCylinder(pts);

	//ProductConeData(pts);
	pts = ReadPointsFromFile("cone.txt");
	//WriteData3("cone1.txt", pts);
	FitCone(pts);
	return 0;

}
