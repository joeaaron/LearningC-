#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>

using namespace std;

// 定义点结构体
struct Point3D {
	double x, y, z;
};

// 定义代价函数
struct RectangleFittingCostFunctor {
	RectangleFittingCostFunctor(const Point3D& point) : point_(point) {}

	template <typename T>
	bool operator()(const T* const rect_params, T* residual) const {
		// 定义输入参数
		Eigen::Matrix<T, 3, 1> p(T(point_.x), T(point_.y), T(point_.z));
		Eigen::Matrix<T, 3, 1> c(rect_params[0], rect_params[1], rect_params[2]);
		Eigen::Matrix<T, 3, 1> u(rect_params[3], rect_params[4], rect_params[5]);
		Eigen::Matrix<T, 3, 1> normal(rect_params[8], rect_params[9], rect_params[10]);

		T length_u = ceres::abs(rect_params[6]);  // 矩形长度
		T length_v = ceres::abs(rect_params[7]);  // 矩形宽度

		// 归一化向量
		normal.normalize();
		u = (u - (u.dot(normal) * normal)).normalized();  // 矩形的U轴归一化
		Eigen::Matrix<T, 3, 1> v = normal.cross(u);  // 矩形的V轴，保证垂直于U和法向量

		// 投影点到矩形平面的距离
		T distance_to_plane = (p - c).dot(normal);

		// 计算投影点
		Eigen::Matrix<T, 3, 1> proj_point = p - distance_to_plane * normal;

		// 矩形中心到投影点的向量
		Eigen::Matrix<T, 3, 1> c_to_proj = proj_point - c;

		// 在矩形边上的投影分量
		T u_proj = c_to_proj.dot(u);
		T v_proj = c_to_proj.dot(v);

		// 半长度和半宽度
		T half_length_u = length_u / T(2.0);
		T half_length_v = length_v / T(2.0);

		// 判定条件
		T u_dist, v_dist;

		if (ceres::abs(u_proj) <= half_length_u && ceres::abs(v_proj) <= half_length_v) {
			// 条件 1: 点在矩形内，计算到矩形最近边的距离
			u_dist = T(1.0) * (half_length_u - ceres::abs(u_proj));
			v_dist = T(1.0) * (half_length_v - ceres::abs(v_proj));
			residual[0] = ceres::sqrt(distance_to_plane * distance_to_plane + std::min(u_dist * u_dist, v_dist * v_dist));
		}
		else if (ceres::abs(u_proj) > half_length_u && ceres::abs(v_proj) > half_length_v) {
			// 条件 2: 点在矩形对角外部，计算到矩形角点的距离
			u_dist = T(1.0) * (ceres::abs(u_proj) - half_length_u);
			v_dist = T(1.0) * (ceres::abs(v_proj) - half_length_v);
			residual[0] = ceres::sqrt(distance_to_plane * distance_to_plane + u_dist * u_dist + v_dist * v_dist);
		}
		else if (ceres::abs(u_proj) > half_length_u) {
			// 条件 3: 点在宽度边界外但在长度内，计算到宽边的距离
			u_dist = T(1.0) * (ceres::abs(u_proj) - half_length_u);
			residual[0] = ceres::sqrt(distance_to_plane * distance_to_plane + u_dist * u_dist);
		}
		else {
			// 条件 4: 点在长度边界外但在宽度内，计算到长边的距离
			v_dist = T(1.0) * (ceres::abs(v_proj) - half_length_v);
			residual[0] = ceres::sqrt(distance_to_plane * distance_to_plane + v_dist * v_dist);
		}

		return true;
	}

private:
	Point3D point_;
};

// 定义代价函数
struct RectangleFittingCost {
	RectangleFittingCost(const Point3D& point) : point_(point) {}

	template <typename T>
	bool operator()(const T* const rect_params, T* residual) const {
		// 矩形参数：[cx, cy, cz, ux, uy, uz, length_u, length_v, normal_x, normal_y, normal_z]
		// cx, cy, cz: 矩形中心
		// ux, uy, uz: 长方向向量
		// length_u: 长度
		// length_v: 宽度
		// normal_x, normal_y, normal_z: 法向量

		Eigen::Matrix<T, 3, 1> p(T(point_.x), T(point_.y), T(point_.z));
		Eigen::Matrix<T, 3, 1> c(rect_params[0], rect_params[1], rect_params[2]);
		Eigen::Matrix<T, 3, 1> u(rect_params[3], rect_params[4], rect_params[5]);
		Eigen::Matrix<T, 3, 1> normal(rect_params[8], rect_params[9], rect_params[10]);

		T length_u = rect_params[6];
		T length_v = rect_params[7];
	
		// 计算宽方向向量
		Eigen::Matrix<T, 3, 1> v = normal.normalized().cross(u);

		T s = (p - c).dot(u);		// 在长度方向上的投影
		T t = (p - c).dot(v);		// 在宽度方向上的投影

		//T dl = ceres::abs(s - length_u * 0.5);
		//T dw = ceres::abs(t - length_v * 0.5);

		//residual[0] = ceres::sqrt((distance_to_plane * distance_to_plane) + std::min(dl, dw) * std::min(dl, dw));

		s = std::max(T(-length_u * 0.5), std::min(s, T(length_u * 0.5)));
		t = std::max(T(-length_v * 0.5), std::min(t, T(length_v * 0.5)));

		Eigen::Matrix<T, 3, 1> projPoint = c + s * u + t * v;
		residual[0] = (projPoint - p).norm();
		//// 点到矩形平面的距离
		//T distance_to_plane = (p - c).dot(normal);

		//// 投影到矩形平面上的点
		//Eigen::Matrix<T, 3, 1> proj_point = p - distance_to_plane * normal;

		//// 矩形中心到投影点的向量
		//Eigen::Matrix<T, 3, 1> c_to_proj = proj_point - c;

		//// 在矩形边上的投影
		//T u_proj = c_to_proj.dot(u.normalized());
		//T v_proj = c_to_proj.dot(v.normalized());

		//// 点到矩形边的距离
		//T u_dist = std::max(T(0), ceres::abs(u_proj) - length_u / T(2));
		//T v_dist = std::max(T(0), ceres::abs(v_proj) - length_v / T(2));
		//
		//residual[0] = ceres::sqrt(distance_to_plane * distance_to_plane + u_dist * u_dist + v_dist * v_dist);

		return true;
	}

private:
	Point3D point_;
};

// 从PCL点云计算初始矩形参数
void ComputeInitialGuess(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
	Eigen::Vector3d& centroid, Eigen::Vector3d& u, double& length_u, double& length_v, Eigen::Vector3d& normal) {
	// 计算质心
	Eigen::Vector4f pcl_centroid;
	pcl::compute3DCentroid(*cloud, pcl_centroid);
	centroid = Eigen::Vector3d(pcl_centroid[0], pcl_centroid[1], pcl_centroid[2]);

	// 计算协方差矩阵
	Eigen::Matrix3f covariance_matrix;
	pcl::computeCovarianceMatrix(*cloud, pcl_centroid, covariance_matrix);

	// 特征值分解
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance_matrix);
	Eigen::Vector3f eigenvalues = solver.eigenvalues();
	Eigen::Matrix3f eigenvectors = solver.eigenvectors();

	// 长方向向量
	u = eigenvectors.col(2).cast<double>(); // 主方向向量

	// 计算初始长宽
	double min_u = std::numeric_limits<double>::max();
	double max_u = std::numeric_limits<double>::lowest();
	double min_v = std::numeric_limits<double>::max();
	double max_v = std::numeric_limits<double>::lowest();

	for (const auto& point : cloud->points) {
		Eigen::Vector3d p(point.x, point.y, point.z);
		Eigen::Vector3d centered = p - centroid;
		double proj_u = centered.dot(u);

		if (proj_u < min_u) min_u = proj_u;
		if (proj_u > max_u) max_u = proj_u;
	}

	length_u = max_u - min_u;

	// 初始法向量
	normal = eigenvectors.col(0).cast<double>();

	// 计算初始宽度
	for (const auto& point : cloud->points) {
		Eigen::Vector3d p(point.x, point.y, point.z);
		Eigen::Vector3d centered = p - centroid;
		double proj_v = centered.dot(normal.cross(u).normalized());

		if (proj_v < min_v) min_v = proj_v;
		if (proj_v > max_v) max_v = proj_v;
	}

	length_v = max_v - min_v;
}

int main() {
	// 读取点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("edge1.pcd", *cloud) == -1) {
		PCL_ERROR("Couldn't read file your_point_cloud.pcd\n");
		return -1;
	}

	// 初始猜测的矩形参数
	Eigen::Vector3d centroid;
	Eigen::Vector3d u;
	Eigen::Vector3d normal;
	double length_u, length_v;

	// 计算初始猜测值
	ComputeInitialGuess(cloud, centroid, u, length_u, length_v, normal);
	normal.normalize();

	//pcl::PointXYZ minPt, maxPt;
	//pcl::getMinMax3D(*cloud, minPt, maxPt);
	//length_v = (maxPt.x - minPt.x);
	//length_u = (maxPt.z - minPt.z);

	double rect_params[11] = {
		centroid[0], centroid[1], centroid[2],  // 中心
		u[0], u[1], u[2],                       // 长方向向量
		length_u,                               // 长度
		length_v,                               // 宽度
		normal[0], normal[1], normal[2]        // 初始法向量
	};

	// 构建Ceres问题
	ceres::Problem problem;
	for (const auto& point : cloud->points) {
		Point3D pt = { point.x, point.y, point.z };
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<RectangleFittingCostFunctor, 1, 11>(
				new RectangleFittingCostFunctor(pt)
			),
			nullptr,
			rect_params
		);
	}

	// 配置求解器
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	options.function_tolerance = 1e-6;					// 函数容差
	options.gradient_tolerance = 1e-6;					// 梯度容差
	options.parameter_tolerance = 1e-6;					// 参数容差
	options.num_threads = 4;

	// 求解问题
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	ceres::Solve(options, &problem, &summary);
	chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
	cout << "Point size:" << cloud->points.size() << endl;
	cout << "Solve time cost = " << time_used.count() << " seconds. " << endl;

	// 输出结果
	std::cout << summary.BriefReport() << std::endl;
	std::cout << "优化后的矩形参数：" << std::endl;
	double norm1 = std::sqrt(rect_params[3] * rect_params[3] + rect_params[4] * rect_params[4] + rect_params[5] * rect_params[5]);
	double norm2 = std::sqrt(rect_params[8] * rect_params[8] + rect_params[9] * rect_params[9] + rect_params[10] * rect_params[10]);
	rect_params[3] /= norm1; rect_params[4] /= norm1; rect_params[5] /= norm1;
	rect_params[8] /= norm2;rect_params[9] /= norm2;rect_params[10] /= norm2;

	//for (int i = 0; i < 11; ++i) {
	//	std::cout << rect_params[i] << " ";
	//}
	//std::cout << std::endl;

	std::cout << "中心: (" << rect_params[0] << ", " << rect_params[1] << ", " << rect_params[2] << ")\n";
	std::cout << "长度方向: (" << rect_params[3] << ", " << rect_params[4] << ", " << rect_params[5] << ")\n";
	std::cout << "轴方向: (" << rect_params[8] << ", " << rect_params[9] << ", " << rect_params[10] << ")\n";
	std::cout << "长度:	" << rect_params[6] << "\n";
	std::cout << "宽度:	" << rect_params[7] << "\n";
	return 0;
}