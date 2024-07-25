#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>

// �����ṹ��
struct Point3D {
	double x, y, z;
};

// ������ۺ���
struct RectangleFittingCost {
	RectangleFittingCost(const Point3D& point, const Eigen::Vector3d& normal)
		: point_(point), normal_(normal) {}

	template <typename T>
	bool operator()(const T* const rect_params, T* residual) const {
		// ���β�����[cx, cy, cz, ux, uy, uz, length_u, length_v]
		// cx, cy, cz: ��������
		// ux, uy, uz: ����������
		// length_u: ����
		// length_v: ���

		Eigen::Matrix<T, 3, 1> p(T(point_.x), T(point_.y), T(point_.z));
		Eigen::Matrix<T, 3, 1> c(rect_params[0], rect_params[1], rect_params[2]);
		Eigen::Matrix<T, 3, 1> u(rect_params[3], rect_params[4], rect_params[5]);

		T length_u = rect_params[6];
		T length_v = rect_params[7];

		// �����������
		Eigen::Matrix<T, 3, 1> r(T(normal_(0)), T(normal_(1)), T(normal_(2)));
		Eigen::Matrix<T, 3, 1> v = r.cross(u).normalized();

		// �㵽����ƽ��ľ���
		T distance_to_plane = (p - c).dot(r);

		// ͶӰ������ƽ���ϵĵ�
		Eigen::Matrix<T, 3, 1> proj_point = p - distance_to_plane * r;

		// �������ĵ�ͶӰ�������
		Eigen::Matrix<T, 3, 1> c_to_proj = proj_point - c;

		// �ھ��α��ϵ�ͶӰ
		T u_proj = c_to_proj.dot(u.normalized());
		T v_proj = c_to_proj.dot(v.normalized());

		// �㵽���αߵľ���
		T u_dist = std::max(T(0), ceres::abs(u_proj) - length_u / T(2));
		T v_dist = std::max(T(0), ceres::abs(v_proj) - length_v / T(2));

		residual[0] = ceres::sqrt(distance_to_plane * distance_to_plane + u_dist * u_dist + v_dist * v_dist);

		return true;
	}

private:
	Point3D point_;
	Eigen::Vector3d normal_;
};

// ��PCL���Ƽ����ʼ���β���
void computeInitialGuess(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Eigen::Vector3d& centroid, Eigen::Vector3d& u, double& length_u, double& length_v) {
	// ��������
	Eigen::Vector4f pcl_centroid;
	pcl::compute3DCentroid(*cloud, pcl_centroid);
	centroid = Eigen::Vector3d(pcl_centroid[0], pcl_centroid[1], pcl_centroid[2]);

	// ����Э�������
	Eigen::Matrix3f covariance_matrix;
	pcl::computeCovarianceMatrix(*cloud, pcl_centroid, covariance_matrix);

	// ����ֵ�ֽ�
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance_matrix);
	Eigen::Vector3f eigenvalues = solver.eigenvalues();
	Eigen::Matrix3f eigenvectors = solver.eigenvectors();

	// ����������
	u = eigenvectors.col(2).cast<double>(); // ����������

	// �����ʼ����
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

	// ʹ��PCA����ĵڶ���������Ϊ����
	Eigen::Vector3d v = eigenvectors.col(1).cast<double>();

	// �����ʼ���
	for (const auto& point : cloud->points) {
		Eigen::Vector3d p(point.x, point.y, point.z);
		Eigen::Vector3d centered = p - centroid;
		double proj_v = centered.dot(v);

		if (proj_v < min_v) min_v = proj_v;
		if (proj_v > max_v) max_v = proj_v;
	}

	length_v = max_v - min_v;
}

int main() 
{
	// ��ȡ��������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("rect1.pcd", *cloud) == -1) {
		PCL_ERROR("Couldn't read file your_point_cloud.pcd\n");
		return -1;
	}

	// ��֪�ķ�����
	Eigen::Vector3d normal(0.003, -1.000, -0.005);

	// ��ʼ�²�ľ��β���
	Eigen::Vector3d centroid;
	Eigen::Vector3d u;
	double length_u, length_v;

	// �����ʼ�²�ֵ
	computeInitialGuess(cloud, centroid, u, length_u, length_v);

	double rect_params[8] = {
		centroid[0], centroid[1], centroid[2],  // ����
		u[0], u[1], u[2],                       // ����������
		length_u,                               // ����
		length_v                                // ���
	};

	// ����Ceres����
	ceres::Problem problem;
	for (const auto& point : cloud->points) {
		Point3D pt = { point.x, point.y, point.z };
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<RectangleFittingCost, 1, 8>(
				new RectangleFittingCost(pt, normal)
			),
			nullptr,
			rect_params
		);
	}

	// ���������
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	options.function_tolerance = 1e-6;					// �����ݲ�
	options.gradient_tolerance = 1e-6;					// �ݶ��ݲ�
	options.parameter_tolerance = 1e-6;					// �����ݲ�
	options.num_threads = 4;

	// �������
	ceres::Solver::Summary summary;
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	ceres::Solve(options, &problem, &summary);
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
	
	double norm = std::sqrt(rect_params[3] * rect_params[3] + rect_params[4] * rect_params[4] + rect_params[5] * rect_params[5]);
	rect_params[3] /= norm;
	rect_params[4] /= norm;
	rect_params[5] /= norm;

	// ������
	std::cout << "Point size:" << cloud->points.size() << std::endl;
	std::cout << "Solve time cost = " << time_used.count() << " seconds. " << std::endl;

	std::cout << summary.BriefReport() << std::endl;
	std::cout << "����: (" << rect_params[0] << ", " << rect_params[1] << ", " << rect_params[2] << ")\n";
	std::cout << "���ȷ���: (" << rect_params[3] << ", " << rect_params[4] << ", " << rect_params[5] << ")\n";
	std::cout << "����:	" << rect_params[6] << "\n";
	std::cout << "���:	" << rect_params[7] << "\n";

	std::cout << std::endl;

	return 0;
}