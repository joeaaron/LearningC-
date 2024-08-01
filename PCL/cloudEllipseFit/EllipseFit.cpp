#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_ellipse3d.h>			// 拟合3D椭圆
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace std;

const int FIT_METHOD = 0;

#define ENABLE_DISPLAY 0		// 定义一个宏，用于控制显示状态

// 定义椭圆参数结构体
struct EllipseParameters 
{
	Eigen::Vector3d center; // 椭圆中心
	double a;				// 半长轴
	double b;				// 半短轴
	Eigen::Vector3d normal; // 法向量
	Eigen::Vector3d u;		// 局部u轴

	//EllipseParameters(const Eigen::Vector3d& c, double a_, double b_, const Eigen::Vector3d& n, const Eigen::Vector3d& u)
	//	: center(c), a(a_), b(b_), normal(n), u_axis(u) {}
};

struct EllipseResidual1 {
	EllipseResidual1(const Eigen::Vector3d& point)
		: point_(point) {}

	template <typename T>
	bool operator()(const T* const params, T* residual) const {
		// 计算椭圆平面的正交基向量v
		Eigen::Matrix<T, 3, 1> n(params[5], params[6], params[7]);
		Eigen::Matrix<T, 3, 1> u(params[8], params[9], params[10]);

		n.normalize();
		u = (u - (u.dot(n) * n)).normalized();  // 矩形的U轴归一化
		Eigen::Matrix<T, 3, 1> v = n.cross(u);  // 矩形的V轴，保证垂直于U和法向量

		// 计算点到椭圆的最近点
		Eigen::Matrix<T, 3, 1> c(params[0], params[1], params[2]);
		T theta = T(0.0);
		T min_distance = std::numeric_limits<T>::max();
		for (T t = T(0.0); t < T(2.0 * M_PI); t += T(0.01)) {
			Eigen::Matrix<T, 3, 1> p = c + params[3] * cos(t) * u + params[4] * sin(t) * v;
			T distance = (point_.cast<T>() - p).squaredNorm();
			if (distance < min_distance) {
				min_distance = distance;
				theta = t;
			}
		}

		// 计算最近点的残差
		Eigen::Matrix<T, 3, 1> p = c + params[3] * cos(theta) * u + params[4] * sin(theta) * v;
		residual[0] = point_[0] - p[0];
		residual[1] = point_[1] - p[1];
		residual[2] = point_[2] - p[2];

		return true;
	}
	static ceres::CostFunction* Create(const Eigen::Vector3d& point) {
		return (new ceres::AutoDiffCostFunction<EllipseResidual1, 1, 11>(
			new EllipseResidual1(point)));
	}
private:
	const Eigen::Vector3d point_;
};

struct EllipseResidual {
	EllipseResidual(const Eigen::Vector3d& point) : point_(point) {}

	template <typename T>
	bool operator()(const T* const center,
		const T* const a,
		const T* const b,
		const T* const normal,
		const T* const u_axis,
		T* residuals) const 
	{
		// 将输入参数转换为Eigen类型
		Eigen::Matrix<T, 3, 1> center_(center[0], center[1], center[2]);
		Eigen::Matrix<T, 3, 1> normal_(normal[0], normal[1], normal[2]);
		Eigen::Matrix<T, 3, 1> u_axis_(u_axis[0], u_axis[1], u_axis[2]);

		// 点到椭圆的距离计算
		Eigen::Matrix<T, 3, 1> diff = point_.cast<T>() - center_;

		// 旋转u_axis，使其与diff对齐
		Eigen::Matrix<T, 3, 1> v_axis = normal_.cross(u_axis_).normalized();
		Eigen::Matrix<T, 3, 1> rotated_diff = Eigen::Matrix<T, 3, 1>(diff.dot(u_axis_), diff.dot(v_axis), diff.dot(normal_));

		// 椭圆方程
		T ellipse_distance = (rotated_diff[0] * rotated_diff[0]) / (a[0] * a[0]) +
			(rotated_diff[1] * rotated_diff[1]) / (b[0] * b[0]);

		residuals[0] = ellipse_distance - T(1.0);

		return true;
	}

	static ceres::CostFunction* Create(const Eigen::Vector3d& point) {
		return (new ceres::AutoDiffCostFunction<EllipseResidual, 3, 3, 1, 1, 3, 3>(
			new EllipseResidual(point)));
	}

	Eigen::Vector3d point_;
};

struct EllipseResidualX {
	EllipseResidualX(const Eigen::Vector3d& point) : point_(point) {}

	template <typename T>
	bool operator()(const T* const params, T* residuals) const 
	{
		Eigen::Matrix<T, 3, 1> center(params[0], params[1], params[2]);
		T a = params[3];
		T b = params[4];
		Eigen::Matrix<T, 3, 1> normal(params[5], params[6], params[7]);
		Eigen::Matrix<T, 3, 1> u_axis(params[8], params[9], params[10]);

		// 计算点到中心的向量
		Eigen::Matrix<T, 3, 1> diff = point_.cast<T>() - center;

		// 计算法向量的正交向量
		normal.normalize();
		u_axis = (u_axis - (u_axis.dot(normal) * normal)).normalized();  // 矩形的U轴归一化
		Eigen::Matrix<T, 3, 1> v_axis = normal.cross(u_axis);  // 矩形的V轴，保证垂直于U和法向量

		// 投影点到矩形平面的距离
		T distance_to_plane = (diff).dot(normal);

		// 旋转u_axis，使其与diff对齐
		Eigen::Matrix<T, 3, 1> rotated_diff(diff.dot(u_axis), diff.dot(v_axis), diff.dot(normal));  //使用这三个向量（u轴、v轴和法向量）将diff向量旋转到一个新的坐标系（这实际上是找到diff在椭圆主轴坐标系下的坐标）。

		// 椭圆方程
		T ellipse_distance = (rotated_diff[0] * rotated_diff[0]) / (a * a) +
			(rotated_diff[1] * rotated_diff[1]) / (b * b);

		T dist = ellipse_distance - T(1.0);
		//residuals[0] = ellipse_distance - T(1.0);
		residuals[0] = ceres::sqrt(distance_to_plane * distance_to_plane + dist * dist);
		return true;
	}

	static ceres::CostFunction* Create(const Eigen::Vector3d& point) {
		return (new ceres::AutoDiffCostFunction<EllipseResidualX, 1, 11>(
			new EllipseResidualX(point)));
	}

	Eigen::Vector3d point_;
};

// 定义点结构体
struct EllipseFittingCostFunctor {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EllipseFittingCostFunctor(const Eigen::Vector3d& point) : point_(point) {}

	template <typename T>
	bool operator()(const T* const ellipse_params, T* residual) const {
		// 定义输入参数
		Eigen::Matrix<T, 3, 1> p(T(point_.x()), T(point_.y()), T(point_.z()));
		Eigen::Matrix<T, 3, 1> c(ellipse_params[0], ellipse_params[1], ellipse_params[2]);
		Eigen::Matrix<T, 3, 1> normal(ellipse_params[5], ellipse_params[6], ellipse_params[7]);
		Eigen::Matrix<T, 3, 1> u(ellipse_params[8], ellipse_params[9], ellipse_params[10]);

		T length_u = ceres::abs(ellipse_params[3]);  // 椭圆长轴
		T length_v = ceres::abs(ellipse_params[4]);  // 椭圆短轴

		// 归一化向量
		normal.normalize();
		u = (u - (u.dot(normal) * normal)).normalized();  
		Eigen::Matrix<T, 3, 1> v = normal.cross(u);  

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
		T half_length_u = length_u;
		T half_length_v = length_v;

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

	static ceres::CostFunction* Create(const Eigen::Vector3d& point) {
		return (new ceres::AutoDiffCostFunction<EllipseFittingCostFunctor, 1, 11>(
			new EllipseFittingCostFunctor(point)));
	}

private:
	Eigen::Vector3d point_;

};

struct EllipseResidualXX {
	EllipseResidualXX(const Eigen::Vector3d& point) : point_(point) {}

	template <typename T>
	bool operator()(const T* const params, T* residuals) const 
	{
		Eigen::Matrix<T, 3, 1> c(params[0], params[1], params[2]);
		T A = params[3];
		T B = params[4];
		Eigen::Matrix<T, 3, 1> n(params[5], params[6], params[7]);
		Eigen::Matrix<T, 3, 1> u(params[8], params[9], params[10]);

		// 计算法向量的正交向量
		Eigen::Matrix<T, 3, 1> v = n.cross(u).normalized();

		// 计算点到中心的向量
		Eigen::Matrix<T, 3, 1> diff = point_.cast<T>() - c;

		// 将diff旋转到椭圆的本地坐标系
		Eigen::Matrix<T, 3, 1> local_diff(u.dot(diff), v.dot(diff), n.dot(diff));

		// 计算到椭圆表面的距离
		T local_distance = ceres::sqrt(local_diff[0] * local_diff[0] / (A * A) +
			local_diff[1] * local_diff[1] / (B * B));

		// 残差为点到椭圆表面的最近距离
		residuals[0] = local_diff.norm() - local_distance;

		return true;
	}

	static ceres::CostFunction* Create(const Eigen::Vector3d& point) {
		return (new ceres::AutoDiffCostFunction<EllipseResidualXX, 1, 11>(
			new EllipseResidualXX(point)));
	}

	Eigen::Vector3d point_;
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

int main()
{
	// -------------------------加载点云------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("ellipse1.pcd", *cloud) < 0)
	{
		PCL_ERROR("Couldn't read file \n");
		return -1;
	}
	// ------------------------RANSAC框架-----------------------------   
	pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>::Ptr Ellipse3D(new pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(Ellipse3D);
	ransac.setDistanceThreshold(0.13);	        // 距离阈值，与模型距离小于0.01的点作为内点
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
	//EllipseParameters ellipse;
	//ellipse.center << static_cast<double>(coeff[0]),
	//	static_cast<double>(coeff[1]),
	//	static_cast<double>(coeff[2]);
	//ellipse.normal << static_cast<double>(coeff[5]),
	//	static_cast<double>(coeff[6]),
	//	static_cast<double>(coeff[7]);
	//ellipse.u << static_cast<double>(coeff[8]),
	//	static_cast<double>(coeff[9]),
	//	static_cast<double>(coeff[10]);
	//ellipse.a = coeff[3];
	//ellipse.b = coeff[4];

	double params[11] = {
	   coeff[0], coeff[1], coeff[2],
	   coeff[3], coeff[4],
	   coeff[5], coeff[6], coeff[7],
	   coeff[8], coeff[9], coeff[10],
	};

	for (const auto& point : points) 
	{
		if (0 == FIT_METHOD)
		{
			ceres::CostFunction* cost_function = EllipseResidualX::Create(point);
			problem.AddResidualBlock(cost_function, nullptr, params);
		}
		else if (1 == FIT_METHOD)
		{
			ceres::CostFunction* cost_function = EllipseResidual1::Create(point);
			problem.AddResidualBlock(cost_function, nullptr, params);
		}	
		else if (2 == FIT_METHOD)
		{
			ceres::CostFunction* cost_function = EllipseFittingCostFunctor::Create(point);
			problem.AddResidualBlock(cost_function, nullptr, params);
		}
	}

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	options.num_threads = 4;					 

	ceres::Solver::Summary summary;
	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	ceres::Solve(options, &problem, &summary);
	chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
	cout << "Point size:" << cloud->points.size() << endl;
	cout << "Solve time cost = " << time_used.count() << " seconds. " << endl;

	double norm = std::sqrt(params[5] * params[5] + params[6] * params[6] + params[7] * params[7]);
	params[5] /= norm;
	params[6] /= norm;
	params[7] /= norm;

	std::cout << summary.BriefReport() << std::endl;
	// 输出拟合结果
	std::cout << "Center: (" << params[0] << ", " << params[1] << ", " << params[2] << ")\n";
	std::cout << "Big diameter: " << params[3] * 2 << "\n";
	std::cout << "Small diameter: " << params[4] * 2 << "\n";
	std::cout << "Normal: (" << params[5] << ", " << params[6] << ", " << params[7] << ")\n";
	std::cout << "U axis: (" << params[8] << ", " << params[9] << ", " << params[10] << ")\n";

#if ENABLE_DISPLAY
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
#endif
	return 0;
}

