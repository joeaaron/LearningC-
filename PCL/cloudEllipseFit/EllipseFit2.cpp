#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_ellipse3d.h>// ���3D��Բ
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace std;

const int FIT_METHOD = 0;

// ������Բ�����ṹ��
struct EllipseParameters {
	double center[3];  // ��Բ���� (x0, y0, z0)
	double a;          // �볤��
	double b;          // �����
	double normal[3];  // ������ (nx, ny, nz)
	double u[3];       // �ֲ�u�� (ux, uy, uz)
};

// ����в��
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
		// ������Բƽ�������������v
		Eigen::Matrix<T, 3, 1> n(normal);
		Eigen::Matrix<T, 3, 1> u_vec(u);
		Eigen::Matrix<T, 3, 1> v = n.cross(u_vec).normalized();

		// ����㵽��Բ�������
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

		// ���������Ĳв�
		Eigen::Matrix<T, 3, 1> p = c + *a * cos(theta) * u_vec + *b * sin(theta) * v;
		residual[0] = point_[0] - p[0];
		residual[1] = point_[1] - p[1];
		residual[2] = point_[2] - p[2];

		return true;
	}

private:
	const Eigen::Vector3d point_;
};

// ����в��-SACMODELELLIPSE
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

// LM�㷨
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
		// ������Բƽ�������������v
		Eigen::Matrix<T, 3, 1> n(normal);
		Eigen::Matrix<T, 3, 1> u_vec(u);
		Eigen::Matrix<T, 3, 1> v = n.cross(u_vec).normalized();

		// ����㵽��Բ�������
		Eigen::Matrix<T, 3, 1> c(center);
		T theta = T(0.0);
		T min_distance = std::numeric_limits<T>::max();

		// LM������
		const T tolerance = T(1e-6);
		const int max_iterations = 100;
		T lambda = T(1e-3);

		for (int i = 0; i < max_iterations; ++i) {
			// ���㵱ǰ��
			Eigen::Matrix<T, 3, 1> p = c + (*a * cos(theta) * u_vec) + (*b * sin(theta) * v);
			Eigen::Matrix<T, 3, 1> dp_dtheta = -(*a * sin(theta) * u_vec) + (*b * cos(theta) * v);

			// ���㵱ǰ������ݶ�
			Eigen::Matrix<T, 3, 1> diff = p - point_.cast<T>();
			T distance = diff.squaredNorm();
			T grad = 2.0 * diff.dot(dp_dtheta);

			// LM������
			T hessian = 2.0 * (dp_dtheta.dot(dp_dtheta) + diff.dot(-(*a * cos(theta) * u_vec) - (*b * sin(theta) * v)));
			T step = grad / (hessian + lambda);
			theta -= step;

			// ����lambda
			if (distance < min_distance) {
				min_distance = distance;
				lambda *= 0.1;
			}
			else {
				lambda *= 10;
			}

			// �����������
			if (step < tolerance) {
				break;
			}
		}

		// ������ѽǶ��µĲв�
		Eigen::Matrix<T, 3, 1> p = c + (*a * cos(theta) * u_vec) + (*b * sin(theta) * v);
		residual[0] = point_[0] - p[0];
		residual[1] = point_[1] - p[1];
		residual[2] = point_[2] - p[2];

		return true;
	}

private:
	const Eigen::Vector3d point_;
};

// ţ�ٷ�
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
		// ������Բƽ�������������v
		Eigen::Matrix<T, 3, 1> n(normal);
		Eigen::Matrix<T, 3, 1> u_vec(u);
		Eigen::Matrix<T, 3, 1> v = n.cross(u_vec).normalized();

		// ����㵽��Բ�������
		Eigen::Matrix<T, 3, 1> c(center);
		T theta = T(0.0);
		T min_distance = std::numeric_limits<T>::max();

		// ��ʼֵ��ţ�ٷ�����
		T t = T(0.0); // ��ʼ�Ƕ�
		const T tolerance = T(1e-6);
		const int max_iterations = 100;

		for (int i = 0; i < max_iterations; ++i) {
			// ���㵱ǰ��
			Eigen::Matrix<T, 3, 1> p = c + (*a * cos(t) * u_vec) + (*b * sin(t) * v);
			Eigen::Matrix<T, 3, 1> dp_dt = -(*a * sin(t) * u_vec) + (*b * cos(t) * v);

			// ���㵱ǰ������ݶ�
			Eigen::Matrix<T, 3, 1> diff = p - point_.cast<T>();
			T distance = diff.squaredNorm();
			T grad = 2.0 * diff.dot(dp_dt);

			// ţ�ٷ�����
			T hessian = 2.0 * (dp_dt.dot(dp_dt) + diff.dot(-(*a * cos(t) * u_vec) - (*b * sin(t) * v)));
			T step = grad / hessian;
			t -= step;

			// �����������
			if (step < tolerance) {
				break;
			}
		}

		// ������ѽǶ��µĲв�
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

	// ʹ�� OpenMP ���л�ѭ��
#pragma omp parallel for
	for (int i = 0; i < cloud->points.size(); ++i)
	{
		vectors[i] = Eigen::Vector3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
	}

	return vectors;

}

// �Զ���в��
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
	// -------------------------���ص���------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("ellipse2.pcd", *cloud) < 0)
	{
		PCL_ERROR("Couldn't read file \n");
		return -1;
	}
	// ------------------------RANSAC���-----------------------------   
	pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>::Ptr Ellipse3D(new pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(Ellipse3D);
	ransac.setDistanceThreshold(0.19);	        // ������ֵ����ģ�;���С��0.01�ĵ���Ϊ�ڵ�
	ransac.setMaxIterations(100);		        // ����������
	ransac.computeModel();				        // ���3D��Բ
	pcl::IndicesPtr inliers(new vector <int>());// �洢�ڵ�����������
	ransac.getInliers(*inliers);			    // ��ȡ�ڵ��Ӧ������
	// -----------------����������ȡ��Բ�ϵĵ�------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr Ellipse_3D(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *Ellipse_3D);
	Eigen::VectorXf coeff;
	ransac.getModelCoefficients(coeff);
	// -------------------����ռ���Բ�Ĳ���--------------------------
	cout << "��Բ���ĵ�X���꣺" << coeff[0] << "\n"
		<< "��Բ���ĵ�Y���꣺" << coeff[1] << "\n"
		<< "��Բ���ĵ�Z���꣺" << coeff[2] << "\n"
		<< "����Բ�ֲ�u��İ볤�᳤�ȣ�" << coeff[3] << "\n"
		<< "����Բ�ֲ�v��İ���᳤�ȣ�" << coeff[4] << "\n"
		<< "���߷����X����:" << coeff[5] << "\n"
		<< "���߷����Y����:" << coeff[6] << "\n"
		<< "���߷����Z����:" << coeff[7] << "\n"
		<< "��Բ�ֲ�u���X����:" << coeff[8] << "\n"
		<< "��Բ�ֲ�u���Y����:" << coeff[9] << "\n"
		<< "��Բ�ֲ�u���Z����:" << coeff[10] << "\n"
		<< endl;
	
	// ceres�����
	std::vector<Eigen::Vector3d> points = PointCloud2Vector3d(cloud);
	ceres::Problem problem;

	// ��ʼ����Բ����
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
			//	new double[11]); // ���ﴫ��һ�������в������
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

	// ---------------����ڵ㲻�����򲻽��п��ӻ�--------------------
	if (Ellipse_3D->size() == 0)
	{
		cerr << "�������ڵ�!!!" << endl;
	}
	else
	{
		cout << "��Ϻ�ĵ�����" << Ellipse_3D->size();
		//-------------------------������ӻ�--------------------------
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(u8"�����ά��Բ"));
		viewer->setBackgroundColor(255, 255, 255);
		// ԭʼ����
		viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
		// ��ϳ��ĵ���
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

