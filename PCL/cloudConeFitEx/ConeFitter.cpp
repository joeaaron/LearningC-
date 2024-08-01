#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <tuple> 

// ����Բ׶������ģ��
struct ConeCostFunction
{
	ConeCostFunction(const pcl::PointXYZ& point)
		: point_(point) {}

	template <typename T>
	bool operator()(const T* const params, T* residuals) const
	{
		Eigen::Matrix<T, 3, 1> axis_point(params[0], params[1], params[2]);
		Eigen::Matrix<T, 3, 1> axis_direction(params[3], params[4], params[5]);
		T alpha = params[6];

		Eigen::Matrix<T, 3, 1> p(T(point_.x), T(point_.y), T(point_.z));
		Eigen::Matrix<T, 3, 1> p_to_axis = p - axis_point;

		// ͶӰ�����ϵ�����
		T t = p_to_axis.dot(axis_direction);
		Eigen::Matrix<T, 3, 1> projection = t * axis_direction;
		// ͶӰ�����ϵľ���
		T distance = (p_to_axis - projection).norm();

		// ����в�㵽Բ׶����ľ��룩
		residuals[0] = distance - t * ceres::tan(alpha);

		return true;
	}

private:
	const pcl::PointXYZ point_;
};

// ��������׶����ת�Ƕ�
double ConeAngleToSlope(double theta) 
{
	// �����¶�
	double slope = (theta) * 180.0 / M_PI;

	return slope;
}

// ʹ��RANSAC���д����
void GetCoefficientsRANSAC(pcl::ModelCoefficients::Ptr& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	//-------------------------------��ȡԲ׶��ģ��------------------------------
	// Estimate point normals
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50); // Adjust depending on your dataset
	normal_estimator.setNumberOfThreads(4);
	normal_estimator.compute(*normals);

	// Create the segmentation object
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// Optional - set the model type (CONE)
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CONE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setProbability(0.95);		// ��ϱ���
	seg.setNumberOfThreads(12);
	seg.setDistanceThreshold(0.01); // Adjust according to your dataset(Ӱ����ϵ���)

	// Segment the largest cone from the cloud
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0) 
	{
		std::cout << "Could not estimate a cone model for the given dataset." << std::endl;
		return;
	}
}

int main(int argc, char** argv) 
{
	// ��ȡ��������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("ConePart2.pcd", *cloud) == -1)		// ConePart.pcd
	{
		PCL_ERROR("Couldn't read file cone.pcd \n");
		return -1;
	}

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	GetCoefficientsRANSAC(coefficients, cloud);

	//��ȡRANSAC���
	double params[7] = { coefficients->values[0], coefficients->values[1], coefficients->values[2],
						 coefficients->values[3], coefficients->values[4], coefficients->values[5],
						 coefficients->values[6] };
	double norm = std::sqrt(params[3] * params[3] + params[4] * params[4] + params[5] * params[5]);

	params[3] /= norm;
	params[4] /= norm;
	params[5] /= norm;

	ceres::Problem problem;
	for (const auto& point : cloud->points) 
	{
		ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<ConeCostFunction, 1, 7>(new ConeCostFunction(point));
		ceres::LossFunction* loss_function = new::ceres::HuberLoss(1.0);  // ʹ��Huber��ʧ����
		problem.AddResidualBlock(cost_function, nullptr, params);
	}

	ceres::Solver::Options options;
	options.minimizer_type = ceres::TRUST_REGION;
	options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;		// ��ӡ������Ϣ
	options.function_tolerance = 1e-6;					// �����ݲ�
	options.gradient_tolerance = 1e-6;					// �ݶ��ݲ�
	options.parameter_tolerance = 1e-6;					// �����ݲ�
	options.max_num_iterations = 100;					// ���������� part1-276
	options.initial_trust_region_radius = 1e4;			// ���ó�ʼ�������С
	options.num_threads = 4;

	// ������������ԺͲ���
	//options.trust_region_minimizer_iterations_to_output = 1;
	options.min_trust_region_radius = 1e-6;
	options.max_trust_region_radius = 1e10;
	options.initial_trust_region_radius = 1e5;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << "\n";
	std::cout << "Final parameters: ";

	for (const auto& param : params) 
	{
		std::cout << param << " ";
	}
	std::cout << "\n";

	double slope = ConeAngleToSlope(params[6]);
	std::cout << slope;
	return 0;
}