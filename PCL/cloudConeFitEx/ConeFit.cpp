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

//// 定义圆锥参数化模型
//struct ConeCostFunction 
//{
//	ConeCostFunction(const pcl::PointXYZ& point) : point_(point) {}
//
//	template <typename T>
//	bool operator()(const T* const params, T* residuals) const 
//	{
//		// 圆锥的参数：底面顶点 (cx, cy, cz)，圆锥轴方向 (ax, ay, az)，半顶角 alpha
//		const T& cx = params[0];
//		const T& cy = params[1];
//		const T& cz = params[2];
//		const T& ax = params[3];
//		const T& ay = params[4];
//		const T& az = params[5];
//		const T& alpha = params[6];
//
//		// 计算点到圆锥轴的距离和角度
//		// 圆锥顶点P与圆锥上任意一点Q构成的向量PQ与轴线向量PN的夹角为半锥顶角
//		T px = T(point_.x) - cx;
//		T py = T(point_.y) - cy;
//		T pz = T(point_.z) - cz;
//
//		//T dot_product = dx * ax + dy * ay + dz * az;
//		//T distance_to_axis = ceres::sqrt(dx * dx + dy * dy + dz * dz - dot_product * dot_product);
//		//T cone_angle = ceres::atan2(distance_to_axis, dot_product * ceres::cos(alpha));
//		//residuals[0] = cone_angle - alpha;
//		// 
//		// 计算向量PN的长度
//		T pnLength = ceres::sqrt(ax * ax + ay * ay + az * az);
//		T nx = ax / pnLength;
//		T ny = ay / pnLength;
//		T nz = az / pnLength;
//
//		// 计算点积 PQ * PN
//		T dot_product = px * nx + py * ny + pz * nz;
//
//		// 计算PQ和PN的夹角
//		T pqLength = ceres::sqrt(px * px + py * py + pz * pz);
//		T theta = ceres::acos(dot_product / (pqLength * pnLength));
//
//		residuals[0] = theta - alpha;
//
//		// 计算残差
//		//T cosBeta = ceres::cos(alpha);
//		//residuals[0] = dot_product - pqLength * cosBeta;
//
//		return true;
//	}
//
//private:
//	const pcl::PointXYZ point_;
//};

// 定义圆锥参数化模型
struct ConeCostFunction
{
	ConeCostFunction(const pcl::PointXYZ& point) 
		: point_(point) {}

	template <typename T>
	bool operator()(const T* const params, T* residuals) const
	{
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

		T es = py * ax;
		T fs = px * ax + py * ay + pz * az;
		//ceres::norm();
		T semiAngle = ceres::abs(alpha);
		T sinSemiAngle = ceres::sin(semiAngle);
		T cosSemiAngle = ceres::cos(semiAngle);

		residuals[0] = fs * sinSemiAngle - es * cosSemiAngle;

		return true;
	}

private:
	const pcl::PointXYZ point_;
	const Eigen::Vector3d coneApexVec_;
};

// 函数：将锥弧度转角度
double ConeAngleToSlope(double theta) 
{
	// 计算坡度
	double slope = (M_PI / 2 - theta) * 180.0 / M_PI;

	return slope;
}

// 使用RANSAC进行粗拟合
void GetCoefficientsRANSAC(pcl::ModelCoefficients::Ptr& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
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

	if (inliers->indices.size() == 0) 
	{
		std::cout << "Could not estimate a cone model for the given dataset." << std::endl;
		return;
	}
}

struct Point3D {
	double x, y, z;
	Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

// 计算质心
Point3D ComputeCentroid(const std::vector<Point3D>& points) 
{
	Point3D centroid = { 0, 0, 0 };
	for (const auto& point : points)
	{
		centroid.x += point.x;
		centroid.y += point.y;
		centroid.z += point.z;
	}
	centroid.x /= points.size();
	centroid.y /= points.size();
	centroid.z /= points.size();
	return centroid;
}

// 计算轴方向
Eigen::Vector3d ComputeAxis(const std::vector<Point3D>& points, const Point3D& centroid) 
{
	Eigen::MatrixXd data(points.size(), 3);
	for (size_t i = 0; i < points.size(); ++i) 
	{
		data(i, 0) = points[i].x - centroid.x;
		data(i, 1) = points[i].y - centroid.y;
		data(i, 2) = points[i].z - centroid.z;
	}

	Eigen::MatrixXd cov = data.transpose() * data;
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
	Eigen::Vector3d axis = eig.eigenvectors().col(2); // 选择最大的特征值对应的特征向量
	return axis;
}

// 计算锥角
double ComputeConeAngle(const std::vector<Point3D>& points, const Point3D& centroid, const Eigen::Vector3d& axis) 
{
	double tan_alpha_sum = 0;
	for (const auto& point : points)
	{
		Eigen::Vector3d P(point.x - centroid.x, point.y - centroid.y, point.z - centroid.z);
		double projectedLength = P.dot(axis);

		Eigen::Vector3d Q = projectedLength * axis;
		double distanceToAxis = (P - Q).norm();

		tan_alpha_sum += distanceToAxis / projectedLength;
	}
	return atan(tan_alpha_sum / points.size());
}

std::vector<Point3D> ConvertPointCloudToVector(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) 
{
	std::vector<Point3D> points;
	points.reserve(cloud->points.size());

	for (const auto& point : cloud->points) {
		points.emplace_back(point.x, point.y, point.z);
	}

	return points;
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

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	GetCoefficientsRANSAC(coefficients, cloud);

	std::vector<Point3D> data_points = ConvertPointCloudToVector(cloud);
	Point3D centroid = ComputeCentroid(data_points);
	Eigen::Vector3d axis = ComputeAxis(data_points, centroid);
	double alpha = ComputeConeAngle(data_points, centroid, axis);

	// 初始化参数: 顶点(Vx, Vy, Vz)，轴方向(Ax, Ay, Az)，锥角(alpha)
	//double params[7] = { centroid.x, centroid.y, centroid.z, axis(0), axis(1), axis(2), alpha };
	// 提取RANSAC结果
	//double params[7] = { coefficients->values[0], coefficients->values[1], coefficients->values[2],
	//					 coefficients->values[3], coefficients->values[4], coefficients->values[5],
	//					 coefficients->values[6] };
	//double norm = std::sqrt(params[3] * params[3] + params[4] * params[4] + params[5] * params[5]);

	//params[3] /= norm;
	//params[4] /= norm;
	//params[5] /= norm;

	double params[7] = {
		0.345, 0.598, 62.091,
		0.000, 0.000306, -1.000,
		0.60
	};

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
	options.minimizer_progress_to_stdout = true;		// 打印迭代信息
	options.function_tolerance = 1e-6;					// 函数容差
	options.gradient_tolerance = 1e-6;					// 梯度容差
	options.parameter_tolerance = 1e-6;					// 参数容差
	options.max_num_iterations = 100;					// 最大迭代次数 part1-276
	options.initial_trust_region_radius = 1e4;			// 设置初始信赖域大小
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