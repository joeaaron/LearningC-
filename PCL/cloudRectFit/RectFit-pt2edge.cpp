#include <ceres/ceres.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm> // For std::max and std::min

// ����㵽�߶ε���̾���
template <typename T>
T PointToSegmentDistance(const Eigen::Matrix<T, 3, 1>& p, const Eigen::Matrix<T, 3, 1>& a, const Eigen::Matrix<T, 3, 1>& b) {
	Eigen::Matrix<T, 3, 1> ab = b - a;
	Eigen::Matrix<T, 3, 1> ap = p - a;
	Eigen::Matrix<T, 3, 1> bp = p - b;
	T ab_ab = ab.dot(ab);
	T ap_ab = ap.dot(ab);
	T t = ap_ab / ab_ab;
	if (t < T(0)) {
		return ap.norm();
	}
	else if (t > T(1)) {
		return bp.norm();
	}
	else {
		Eigen::Matrix<T, 3, 1> closest_point = a + t * ab;
		return (p - closest_point).norm();
	}
}

// �����Ż�����
struct RectangleCostFunction {
	RectangleCostFunction(const pcl::PointXYZ& point, const Eigen::Vector3d& initial_center, const Eigen::Vector3d& initial_v1, const Eigen::Vector3d& initial_v2)
		: point_(point), initial_center_(initial_center), initial_v1_(initial_v1), initial_v2_(initial_v2) {}

	template <typename T>
	bool operator()(const T* const center, const T* const v1, const T* const v2, const T* const length, const T* const width, T* residual) const {
		// ת���������
		Eigen::Matrix<T, 3, 1> center_eigen(center[0], center[1], center[2]);
		Eigen::Matrix<T, 3, 1> v1_eigen(v1[0], v1[1], v1[2]);
		Eigen::Matrix<T, 3, 1> v2_eigen(v2[0], v2[1], v2[2]);
		T length_t = length[0];
		T width_t = width[0];

		// �����ȷ���
		Eigen::Matrix<T, 3, 1> width_direction = v1_eigen.normalized().cross(v2_eigen);

		// ������ε��ĸ��ǵ�
		Eigen::Matrix<T, 3, 1> p0 = center_eigen - T(length_t * 0.5) * v2_eigen - T(width_t * 0.5) * width_direction;
		Eigen::Matrix<T, 3, 1> p1 = center_eigen + T(length_t * 0.5) * v2_eigen - T(width_t * 0.5) * width_direction;
		Eigen::Matrix<T, 3, 1> p2 = center_eigen + T(length_t * 0.5) * v2_eigen + T(width_t * 0.5) * width_direction;
		Eigen::Matrix<T, 3, 1> p3 = center_eigen - T(length_t * 0.5) * v2_eigen + T(width_t * 0.5) * width_direction;

		// ����㵽���������ߵ���С����
		T min_distance = std::min({
			PointToSegmentDistance<T>(Eigen::Matrix<T, 3, 1>(T(point_.x), T(point_.y), T(point_.z)), p0, p1),
			PointToSegmentDistance<T>(Eigen::Matrix<T, 3, 1>(T(point_.x), T(point_.y), T(point_.z)), p1, p2),
			PointToSegmentDistance<T>(Eigen::Matrix<T, 3, 1>(T(point_.x), T(point_.y), T(point_.z)), p2, p3),
			PointToSegmentDistance<T>(Eigen::Matrix<T, 3, 1>(T(point_.x), T(point_.y), T(point_.z)), p3, p0)
			});

		// ����в�
		residual[0] = min_distance;
		return true;
	}

private:
	const pcl::PointXYZ point_;
	const Eigen::Vector3d initial_center_;
	const Eigen::Vector3d initial_v1_;
	const Eigen::Vector3d initial_v2_;
};

int main(int argc, char** argv) {
	// ���ص�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile("edge1.pcd", *cloud) == -1) {
		PCL_ERROR("Couldn't read file point_cloud.pcd\n");
		return -1;
	}

	// ������Ƶ� PCA
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloud);

	Eigen::Vector4f pca_mean;
	Eigen::Matrix3f pca_eigenvectors;
	pca_mean = pca.getMean();
	pca_eigenvectors = pca.getEigenVectors();

	// ��ȡ PCA ���
	Eigen::Vector3d center(pca_mean[0], pca_mean[1], pca_mean[2]);
	Eigen::Vector3d v1(pca_eigenvectors.col(0).cast<double>()); // �����򣨴��Ż��ķ�������
	Eigen::Vector3d v2(pca_eigenvectors.col(1).cast<double>()); // ���ȷ���

	// �����ȷ���
	//Eigen::Vector3d width_direction = v1.cross(v2);

	// ������εĳ�ʼ���ȺͿ�ȣ����Ը��ݵ��Ƶķֲ����и���ȷ�Ĺ��ƣ�
	Eigen::Vector4f minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);
	//double length = (maxPt[1] - minPt[1]);
	//double width = (maxPt[0] - minPt[0]);

	double length = 38;
	double width = 5;

	// �����Ż�����
	ceres::Problem problem;
	for (const auto& point : cloud->points) {
		ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<RectangleCostFunction, 1, 3, 3, 3, 1, 1>(
			new RectangleCostFunction(point, center, v1, v2));
		problem.AddResidualBlock(cost_function, nullptr, center.data(), v1.data(), v2.data(), &length, &width);
	}

	// ���������
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	// ������
	std::cout << summary.FullReport() << std::endl;
	std::cout << "Optimized center: " << center.transpose() << std::endl;
	std::cout << "Optimized v1 (width direction): " << v1.transpose() << std::endl;
	std::cout << "Optimized v2 (length direction): " << v2.transpose() << std::endl;
	std::cout << "Optimized length: " << length << std::endl;
	std::cout << "Optimized width: " << width << std::endl;

	return 0;
}