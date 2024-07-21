#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <algorithm>

// 矩形模型的参数结构
struct RectangleParameters 
{
	Eigen::Vector3d center;
	double length;
	double width;
	Eigen::Matrix3d rotation;
};

// 残差函数
struct RectangleResidual {
	RectangleResidual(const Eigen::Vector3d& point) : point_(point) {}

	template <typename T>
	bool operator()(const T* const center,
		const T* const length,
		const T* const width,
		const T* const rotation,
		T* residual) const {
		Eigen::Matrix<T, 3, 1> p(T(point_(0)), T(point_(1)), T(point_(2)));
		Eigen::Matrix<T, 3, 1> c(center[0], center[1], center[2]);
		T l = length[0];
		T w = width[0];

		// 旋转矩阵
		Eigen::Matrix<T, 3, 3> R;
		R << rotation[0], rotation[1], rotation[2],
			rotation[3], rotation[4], rotation[5],
			rotation[6], rotation[7], rotation[8];

		// 计算点在矩形平面上的投影
		Eigen::Matrix<T, 3, 1> diff = R.transpose() * (p - c);

		// 计算点到矩形的距离
		T dx = std::max(T(0), ceres::abs(diff[0]) - l / T(2));
		T dy = std::max(T(0), ceres::abs(diff[1]) - w / T(2));
		residual[0] = sqrt(dx * dx + dy * dy + diff[2] * diff[2]);

		return true;
	}

private:
	const Eigen::Vector3d point_;
};

// 使用PCA进行初始估计
RectangleParameters EstimateInitialParameters(const std::vector<Eigen::Vector3d>& points) 
{
	// PCA to determine the main plane
	Eigen::MatrixXd A(points.size(), 3);
	for (size_t i = 0; i < points.size(); ++i) {
		A.row(i) = points[i];
	}

	// Compute the centroid
	Eigen::Vector3d centroid = A.colwise().mean();
	A.rowwise() -= centroid.transpose();

	// PCA compute covariance matrix
	Eigen::Matrix3d cov = A.transpose() * A;

	// Solve for eigenvalues and eigenvectors
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
	Eigen::Vector3d eigenvalues = solver.eigenvalues();
	Eigen::Matrix3d eigenvectors = solver.eigenvectors();

	// 计算矩形的初始边长
	double length = 2 * std::sqrt(eigenvalues(2) / points.size());
	double width = 2 * std::sqrt(eigenvalues(1) / points.size());

	RectangleParameters params;
	params.center = centroid;
	params.length = length;
	params.width = width;
	params.rotation = eigenvectors;

	return params;
}

int main(int argc, char** argv) {
	//google::InitGoogleLogging(argv[0]);

	// 生成点云数据（这里用真实数据替代）
	std::vector<Eigen::Vector3d> points = {
		// 添加点坐标
	};

	// 初始估计
	RectangleParameters params = EstimateInitialParameters(points);

	// 创建Ceres问题
	ceres::Problem problem;
	for (const auto& point : points) {
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<RectangleResidual, 1, 3, 1, 1, 9>(
				new RectangleResidual(point)),
			nullptr,
			params.center.data(),
			&params.length,
			&params.width,
			params.rotation.data()
		);
	}

	// 配置求解器选项
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	// 求解
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	// 输出结果
	std::cout << summary.FullReport() << "\n";
	std::cout << "Estimated center: " << params.center.transpose() << "\n";
	std::cout << "Estimated length: " << params.length << "\n";
	std::cout << "Estimated width: " << params.width << "\n";
	std::cout << "Estimated rotation:\n" << params.rotation << "\n";

	return 0;
}