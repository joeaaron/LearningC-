#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_ellipse3d.h>// 拟合3D椭圆
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace std;

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
	if (pcl::io::loadPCDFile("ellipse.pcd", *cloud) < 0)
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

	for (const auto& point : points) {
		ceres::CostFunction* cost_function =
			new ceres::AutoDiffCostFunction<EllipseResidual, 3, 3, 1, 1, 3, 3>(
				new EllipseResidual(point));
		problem.AddResidualBlock(cost_function, nullptr, ellipse.center, &ellipse.a, &ellipse.b, ellipse.normal, ellipse.u);
	}

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	double norm = std::sqrt(ellipse.normal[0] * ellipse.normal[0] + ellipse.normal[1] * ellipse.normal[1] + ellipse.normal[2] * ellipse.normal[2]);

	ellipse.normal[0] /= norm;
	ellipse.normal[1] /= norm;
	ellipse.normal[2] /= norm;

	std::cout << summary.FullReport() << std::endl;
	std::cout << "Center: (" << ellipse.center[0] << ", " << ellipse.center[1] << ", " << ellipse.center[2] << ")" << std::endl;
	std::cout << "Semi-major axis: " << ellipse.a << std::endl;
	std::cout << "Semi-minor axis: " << ellipse.b << std::endl;
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

