#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// 定义圆锥参数化模型
struct ConeCostFunction 
{
	ConeCostFunction(const pcl::PointXYZ& point) : point_(point) {}

	template <typename T>
	bool operator()(const T* const params, T* residuals) const 
	{
		// 圆锥的参数：底面顶点 (cx, cy, cz)，圆锥轴方向 (ax, ay, az)，半顶角 alpha
		const T& cx = params[0];
		const T& cy = params[1];
		const T& cz = params[2];
		const T& ax = params[3];
		const T& ay = params[4];
		const T& az = params[5];
		const T& alpha = params[6];

		// 计算点到圆锥轴的距离和角度
		// 圆锥顶点P与圆锥上任意一点Q构成的向量PQ与轴线向量PN的夹角为半锥顶角
		T px = T(point_.x) - cx;
		T py = T(point_.y) - cy;
		T pz = T(point_.z) - cz;

		//T dot_product = dx * ax + dy * ay + dz * az;
		//T distance_to_axis = ceres::sqrt(dx * dx + dy * dy + dz * dz - dot_product * dot_product);
		//T cone_angle = ceres::atan2(distance_to_axis, dot_product * ceres::cos(alpha));
		//residuals[0] = cone_angle - alpha;
		// 
		// 计算向量PN的长度
		T pnLength = ceres::sqrt(ax * ax + ay * ay + az * az);
		T nx = ax / pnLength;
		T ny = ay / pnLength;
		T nz = az / pnLength;

		// 计算点积 PQ * PN
		T dot_product = px * nx + py * ny + pz * nz;

		// 计算PQ和PN的夹角
		T pqLength = ceres::sqrt(px * px + py * py + pz * pz);
		T theta = ceres::acos(dot_product / (pqLength * pnLength));

		residuals[0] = theta - alpha;

		// 计算残差
		//T cosBeta = ceres::cos(alpha);
		//residuals[0] = dot_product - pqLength * cosBeta;

		return true;
	}

private:
	const pcl::PointXYZ point_;
};

// 函数：将锥弧度转角度
double ConeAngleToSlope(double theta) 
{
	// 计算坡度
	double slope = (M_PI / 2 - theta) * 180.0 / M_PI;

	return slope;
}


int main(int argc, char** argv) 
{
	// 读取点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("ConePart2.pcd", *cloud) == -1)		// ConePart.pcd
	{
		PCL_ERROR("Couldn't read file cone.pcd \n");
		return -1;
	}

	// 使用RANSAC进行粗拟合
	//-------------------------------提取圆锥体模型------------------------------
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
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

	// Optional - set the model type (CONE)
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CONE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setProbability(0.95);		// 拟合比例
	seg.setNumberOfThreads(12);
	seg.setDistanceThreshold(0.01); // Adjust according to your dataset(影响拟合点数)

	// Segment the largest cone from the cloud
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0) {
		std::cout << "Could not estimate a cone model for the given dataset." << std::endl;
		return -1;
	}

	// 提取RANSAC结果
	double params[7] = { coefficients->values[0], coefficients->values[1], coefficients->values[2],
						 coefficients->values[3], coefficients->values[4], coefficients->values[5],
						 coefficients->values[6] };
	double norm = std::sqrt(params[3] * params[3] + params[4] * params[4] + params[5] * params[5]);

	params[3] /= norm;
	params[4] /= norm;
	params[5] /= norm;

	//double params[7] = {
	//	0.2870005, 0.680849, 62.088072,
	//	0.0009, -0.001306, -0.99999,
	//	0.90
	//};
	// 使用Ceres进行精拟合
	ceres::Problem problem;
	for (const auto& point : cloud->points) 
	{
		ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<ConeCostFunction, 1, 7>(new ConeCostFunction(point));
		ceres::LossFunction* loss_function = new::ceres::HuberLoss(1.0);  // 使用Huber损失函数
		problem.AddResidualBlock(cost_function, nullptr, params);
	}

	ceres::Solver::Options options;
	options.minimizer_type = ceres::TRUST_REGION;
	options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	//options.max_num_iterations = 100;			// 最大迭代次数 part1-276
	options.initial_trust_region_radius = 1e4;	// 设置初始信赖域大小
	options.num_threads = 4;

	// 设置信赖域策略和参数
	//options.trust_region_minimizer_iterations_to_output = 1;
	options.min_trust_region_radius = 1e-6;
	options.max_trust_region_radius = 1e10;
	options.initial_trust_region_radius = 1e5;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.FullReport() << "\n";
	std::cout << "Final parameters: ";
	for (const auto& param : params) {
		std::cout << param << " ";
	}
	std::cout << "\n";

	double slope = ConeAngleToSlope(params[6]);
	std::cout << slope / 2;
	return 0;
}