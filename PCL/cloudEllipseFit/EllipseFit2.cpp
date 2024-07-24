#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_ellipse3d.h>// 拟合3D椭圆
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace std;

const int FIT_METHOD = 0;

// 定义椭圆参数结构体
struct EllipseParameters {
	double center[3];  // 椭圆中心 (x0, y0, z0)
	double a;          // 半长轴
	double b;          // 半短轴
	double normal[3];  // 法向量 (nx, ny, nz)
	double u[3];       // 局部u轴 (ux, uy, uz)
};

// 定义残差块
struct EllipseResidual {
	EllipseResidual(const Eigen::Vector3d& point)
		: point_(point) {}

	template <typename T>
	bool operator()(const T* const center,
		const T* const a,
		const T* const b,
		const T* const normal,
		const T* const u,
		T* residual) const {
		// 计算椭圆平面的正交基向量v
		Eigen::Matrix<T, 3, 1> n(normal);
		Eigen::Matrix<T, 3, 1> u_vec(u);
		Eigen::Matrix<T, 3, 1> v = n.cross(u_vec).normalized();

		// 计算点到椭圆的最近点
		Eigen::Matrix<T, 3, 1> c(center);
		T theta = T(0.0);
		T min_distance = std::numeric_limits<T>::max();
		for (T t = T(0.0); t < T(2.0 * M_PI); t += T(0.01)) {
			Eigen::Matrix<T, 3, 1> p = c + *a * cos(t) * u_vec + *b * sin(t) * v;
			T distance = (point_.cast<T>() - p).squaredNorm();
			if (distance < min_distance) {
				min_distance = distance;
				theta = t;
			}
		}

		// 计算最近点的残差
		Eigen::Matrix<T, 3, 1> p = c + *a * cos(theta) * u_vec + *b * sin(theta) * v;
		residual[0] = point_[0] - p[0];
		residual[1] = point_[1] - p[1];
		residual[2] = point_[2] - p[2];

		return true;
	}

private:
	const Eigen::Vector3d point_;
};

// 定义残差块-SACMODELELLIPSE
struct EllipseResidualSAC
{
	EllipseResidualSAC(const Eigen::Vector3d& point)
		: point_(point) {}

	template <typename T>
	bool operator()(const T* const center,
		const T* const a,
		const T* const b,
		const T* const normal,
		const T* const u,
		T* residual) const 
	{
		// c : Ellipse Center
		Eigen::Matrix<T, 3, 1> c(center);
		// n : Ellipse (Plane) Normal
		Eigen::Matrix<T, 3, 1> n(normal);
		// x : Ellipse (Plane) X-Axis
		Eigen::Matrix<T, 3, 1> x_axis(u);
		// y : Ellipse (Plane) Y-Axis
		Eigen::Matrix<T, 3, 1> y_axis = n.cross(x_axis).normalized();

		// Compute the rotation matrix and its transpose
		Eigen::Matrix<T, 3, 3> Rot;
		Rot << x_axis(0), y_axis(0), n(0),
			x_axis(1), y_axis(1), n(1),
			x_axis(2), y_axis(2), n(2);
		Eigen::Matrix<T, 3, 3> Rot_T = Rot.transpose();

		// Ellipse parameters
		Eigen::Matrix<T, 5, 1> params(a, b, 0, 0, 0);
		//params << par_a, par_b, T(0.0), T(0.0), T(0.0);
		T th_opt;

		// Local coordinates of sample point p
		Eigen::Matrix<T, 3, 1> p_ = Rot_T * (point_ - c);

		// k : Point on Ellipse
		// Calculate the shortest distance from the point to the ellipse
		Eigen::Matrix<T, 2, 1> distanceVector = dvec2ellipse(params, p_(0), p_(1), th_opt);

		// Calculate the residual
		residual[0] = distanceVector.norm();

		return true;
	}

private:
	const Eigen::Vector3d point_;
};

// LM算法
struct EllipseResidualLM {
	EllipseResidualLM(const Eigen::Vector3d& point)
		: point_(point) {}

	template <typename T>
	bool operator()(const T* const center,
		const T* const a,
		const T* const b,
		const T* const normal,
		const T* const u,
		T* residual) const {
		// 计算椭圆平面的正交基向量v
		Eigen::Matrix<T, 3, 1> n(normal);
		Eigen::Matrix<T, 3, 1> u_vec(u);
		Eigen::Matrix<T, 3, 1> v = n.cross(u_vec).normalized();

		// 计算点到椭圆的最近点
		Eigen::Matrix<T, 3, 1> c(center);
		T theta = T(0.0);
		T min_distance = std::numeric_limits<T>::max();

		// LM法参数
		const T tolerance = T(1e-6);
		const int max_iterations = 100;
		T lambda = T(1e-3);

		for (int i = 0; i < max_iterations; ++i) {
			// 计算当前点
			Eigen::Matrix<T, 3, 1> p = c + (*a * cos(theta) * u_vec) + (*b * sin(theta) * v);
			Eigen::Matrix<T, 3, 1> dp_dtheta = -(*a * sin(theta) * u_vec) + (*b * cos(theta) * v);

			// 计算当前距离和梯度
			Eigen::Matrix<T, 3, 1> diff = p - point_.cast<T>();
			T distance = diff.squaredNorm();
			T grad = 2.0 * diff.dot(dp_dtheta);

			// LM法更新
			T hessian = 2.0 * (dp_dtheta.dot(dp_dtheta) + diff.dot(-(*a * cos(theta) * u_vec) - (*b * sin(theta) * v)));
			T step = grad / (hessian + lambda);
			theta -= step;

			// 调整lambda
			if (distance < min_distance) {
				min_distance = distance;
				lambda *= 0.1;
			}
			else {
				lambda *= 10;
			}

			// 检查收敛条件
			if (step < tolerance) {
				break;
			}
		}

		// 计算最佳角度下的残差
		Eigen::Matrix<T, 3, 1> p = c + (*a * cos(theta) * u_vec) + (*b * sin(theta) * v);
		residual[0] = point_[0] - p[0];
		residual[1] = point_[1] - p[1];
		residual[2] = point_[2] - p[2];

		return true;
	}

private:
	const Eigen::Vector3d point_;
};

// 牛顿法
struct EllipseResidualNewton {
	EllipseResidualNewton(const Eigen::Vector3d& point)
		: point_(point) {}

	template <typename T>
	bool operator()(const T* const center,
		const T* const a,
		const T* const b,
		const T* const normal,
		const T* const u,
		T* residual) const {
		// 计算椭圆平面的正交基向量v
		Eigen::Matrix<T, 3, 1> n(normal);
		Eigen::Matrix<T, 3, 1> u_vec(u);
		Eigen::Matrix<T, 3, 1> v = n.cross(u_vec).normalized();

		// 计算点到椭圆的最近点
		Eigen::Matrix<T, 3, 1> c(center);
		T theta = T(0.0);
		T min_distance = std::numeric_limits<T>::max();

		// 初始值和牛顿法参数
		T t = T(0.0); // 初始角度
		const T tolerance = T(1e-6);
		const int max_iterations = 100;

		for (int i = 0; i < max_iterations; ++i) {
			// 计算当前点
			Eigen::Matrix<T, 3, 1> p = c + (*a * cos(t) * u_vec) + (*b * sin(t) * v);
			Eigen::Matrix<T, 3, 1> dp_dt = -(*a * sin(t) * u_vec) + (*b * cos(t) * v);

			// 计算当前距离和梯度
			Eigen::Matrix<T, 3, 1> diff = p - point_.cast<T>();
			T distance = diff.squaredNorm();
			T grad = 2.0 * diff.dot(dp_dt);

			// 牛顿法更新
			T hessian = 2.0 * (dp_dt.dot(dp_dt) + diff.dot(-(*a * cos(t) * u_vec) - (*b * sin(t) * v)));
			T step = grad / hessian;
			t -= step;

			// 检查收敛条件
			if (step < tolerance) {
				break;
			}
		}

		// 计算最佳角度下的残差
		Eigen::Matrix<T, 3, 1> p = c + (*a * cos(t) * u_vec) + (*b * sin(t) * v);
		residual[0] = point_[0] - p[0];
		residual[1] = point_[1] - p[1];
		residual[2] = point_[2] - p[2];

		return true;
	}

private:
	const Eigen::Vector3d point_;
};

std::vector<Eigen::Vector3d> PointCloud2Vector3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::vector<Eigen::Vector3d> vectors(cloud->points.size());

	// 使用 OpenMP 并行化循环
#pragma omp parallel for
	for (int i = 0; i < cloud->points.size(); ++i)
	{
		vectors[i] = Eigen::Vector3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
	}

	return vectors;

}

// 自定义残差块
struct EllipseResidualDef 
{
	EllipseResidualDef(const Eigen::Vector3d& point, const Eigen::VectorXf& model_coefficients)
		: point_(point)
		, model_coefficients_(model_coefficients) {}

	template <typename T>
	bool operator()(T* residual) const 
	{
		// c : Ellipse Center
		const Eigen::Vector3f c(model_coefficients_[0], model_coefficients_[1], model_coefficients_[2]);
		// n : Ellipse (Plane) Normal
		const Eigen::Vector3f n_axis(model_coefficients_[5], model_coefficients_[6], model_coefficients_[7]);
		// x : Ellipse (Plane) X-Axis
		const Eigen::Vector3f x_axis(model_coefficients_[8], model_coefficients_[9], model_coefficients_[10]);
		// y : Ellipse (Plane) Y-Axis
		const Eigen::Vector3f y_axis = n_axis.cross(x_axis).normalized();
		// a : Ellipse semi-major axis (X) length
		const float par_a(model_coefficients_[3]);
		// b : Ellipse semi-minor axis (Y) length
		const float par_b(model_coefficients_[4]);

		// Compute the rotation matrix and its transpose
		const Eigen::Matrix3f Rot = (Eigen::Matrix3f(3, 3)
			<< x_axis(0), y_axis(0), n_axis(0),
			x_axis(1), y_axis(1), n_axis(1),
			x_axis(2), y_axis(2), n_axis(2))
			.finished();
		const Eigen::Matrix3f Rot_T = Rot.transpose();

		// Ellipse parameters
		const Eigen::VectorXf params = (Eigen::VectorXf(5) << par_a, par_b, 0.0, 0.0, 0.0).finished();
		float th_opt;

		const Eigen::Vector3f p_ = Rot_T * (point_ - c);
		const Eigen::Vector2f distanceVector = dvec2ellipse(params, p_(0), p_(1), th_opt);
		residual[0] = = distanceVector.norm();

		return true;
	}

private:
	const Eigen::Vector3d point_;
	const Eigen::VectorXf model_coefficients_;
};

int main()
{
	// -------------------------加载点云------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("ellipse2.pcd", *cloud) < 0)
	{
		PCL_ERROR("Couldn't read file \n");
		return -1;
	}
	// ------------------------RANSAC框架-----------------------------   
	pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>::Ptr Ellipse3D(new pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(Ellipse3D);
	ransac.setDistanceThreshold(0.19);	        // 距离阈值，与模型距离小于0.01的点作为内点
	ransac.setMaxIterations(100);		        // 最大迭代次数
	ransac.computeModel();				        // 拟合3D椭圆
	pcl::IndicesPtr inliers(new vector <int>());// 存储内点索引的向量
	ransac.getInliers(*inliers);			    // 提取内点对应的索引
	// -----------------根据索引提取椭圆上的点------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr Ellipse_3D(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *Ellipse_3D);
	Eigen::VectorXf coeff;
	ransac.getModelCoefficients(coeff);
	// -------------------输出空间椭圆的参数--------------------------
	cout << "椭圆中心的X坐标：" << coeff[0] << "\n"
		<< "椭圆中心的Y坐标：" << coeff[1] << "\n"
		<< "椭圆中心的Z坐标：" << coeff[2] << "\n"
		<< "沿椭圆局部u轴的半长轴长度：" << coeff[3] << "\n"
		<< "沿椭圆局部v轴的半短轴长度：" << coeff[4] << "\n"
		<< "法线方向的X坐标:" << coeff[5] << "\n"
		<< "法线方向的Y坐标:" << coeff[6] << "\n"
		<< "法线方向的Z坐标:" << coeff[7] << "\n"
		<< "椭圆局部u轴的X坐标:" << coeff[8] << "\n"
		<< "椭圆局部u轴的Y坐标:" << coeff[9] << "\n"
		<< "椭圆局部u轴的Z坐标:" << coeff[10] << "\n"
		<< endl;
	
	// ceres精拟合
	std::vector<Eigen::Vector3d> points = PointCloud2Vector3d(cloud);
	ceres::Problem problem;

	// 初始化椭圆参数
	EllipseParameters ellipse;
	ellipse.center[0] = coeff[0];
	ellipse.center[1] = coeff[1];
	ellipse.center[2] = coeff[2];
	ellipse.a = coeff[3];
	ellipse.b = coeff[4];
	ellipse.normal[0] = coeff[5];
	ellipse.normal[1] = coeff[6];
	ellipse.normal[2] = coeff[7];
	ellipse.u[0] = coeff[8];
	ellipse.u[1] = coeff[9];
	ellipse.u[2] = coeff[10];

	for (const auto& point : points) 
	{
		if (0 == FIT_METHOD)
		{
			ceres::CostFunction* cost_function =
				new ceres::AutoDiffCostFunction<EllipseResidual, 3, 3, 1, 1, 3, 3>(
					new EllipseResidual(point));

			problem.AddResidualBlock(cost_function, nullptr, ellipse.center, &ellipse.a, &ellipse.b, ellipse.normal, ellipse.u);
			
			//problem.AddResidualBlock(
			//	new ceres::AutoDiffCostFunction<EllipseResidualDef, 3>(
			//		new EllipseResidualDef(point, coeff)),
			//	nullptr,
			//	new double[11]); // 这里传入一个包含残差的数组
		}	
		else if (1 == FIT_METHOD)
		{
			ceres::CostFunction* cost_function =
				new ceres::AutoDiffCostFunction<EllipseResidualLM, 3, 3, 1, 1, 3, 3>(
					new EllipseResidualLM(point));

			problem.AddResidualBlock(cost_function, nullptr, ellipse.center, &ellipse.a, &ellipse.b, ellipse.normal, ellipse.u);
		}
		else if (2 == FIT_METHOD)
		{
			ceres::CostFunction* cost_function =
				new ceres::AutoDiffCostFunction<EllipseResidualNewton, 3, 3, 1, 1, 3, 3>(
					new EllipseResidualNewton(point));

			problem.AddResidualBlock(cost_function, nullptr, ellipse.center, &ellipse.a, &ellipse.b, ellipse.normal, ellipse.u);
		}
	}

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	options.num_threads = 4;					 

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	double norm = std::sqrt(ellipse.normal[0] * ellipse.normal[0] + ellipse.normal[1] * ellipse.normal[1] + ellipse.normal[2] * ellipse.normal[2]);

	ellipse.normal[0] /= norm;
	ellipse.normal[1] /= norm;
	ellipse.normal[2] /= norm;

	std::cout << summary.BriefReport() << std::endl;
	std::cout << "Center: (" << ellipse.center[0] << ", " << ellipse.center[1] << ", " << ellipse.center[2] << ")" << std::endl;
	std::cout << "Semi-major axis: " << ellipse.a*2 << std::endl;
	std::cout << "Semi-minor axis: " << ellipse.b*2 << std::endl;
	std::cout << "Normal: (" << ellipse.normal[0] << ", " << ellipse.normal[1] << ", " << ellipse.normal[2] << ")" << std::endl;
	std::cout << "U axis: (" << ellipse.u[0] << ", " << ellipse.u[1] << ", " << ellipse.u[2] << ")" << std::endl;

	// ---------------如果内点不存在则不进行可视化--------------------
	if (Ellipse_3D->size() == 0)
	{
		cerr << "不存在内点!!!" << endl;
	}
	else
	{
		cout << "拟合后的点数：" << Ellipse_3D->size();
		//-------------------------结果可视化--------------------------
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(u8"拟合三维椭圆"));
		viewer->setBackgroundColor(255, 255, 255);
		// 原始点云
		viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
		// 拟合出的点云
		viewer->addPointCloud<pcl::PointXYZ>(Ellipse_3D, "elipse3D");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "elipse3D");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "elipse3D");

		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}

	return 0;
}

