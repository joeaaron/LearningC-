#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_ellipse3d.h>			// ���3D��Բ
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace std;

const int FIT_METHOD = 0;

#define ENABLE_DISPLAY 0		// ����һ���꣬���ڿ�����ʾ״̬

// ������Բ�����ṹ��
struct EllipseParameters 
{
	Eigen::Vector3d center; // ��Բ����
	double a;				// �볤��
	double b;				// �����
	Eigen::Vector3d normal; // ������
	Eigen::Vector3d u;		// �ֲ�u��

	//EllipseParameters(const Eigen::Vector3d& c, double a_, double b_, const Eigen::Vector3d& n, const Eigen::Vector3d& u)
	//	: center(c), a(a_), b(b_), normal(n), u_axis(u) {}
};

struct EllipseResidual1 {
	EllipseResidual1(const Eigen::Vector3d& point)
		: point_(point) {}

	template <typename T>
	bool operator()(const T* const params, T* residual) const {
		// ������Բƽ�������������v
		Eigen::Matrix<T, 3, 1> n(params[5], params[6], params[7]);
		Eigen::Matrix<T, 3, 1> u_vec(params[8], params[9], params[10]);
		Eigen::Matrix<T, 3, 1> v = n.cross(u_vec).normalized();

		// ����㵽��Բ�������
		Eigen::Matrix<T, 3, 1> c(params[0], params[1], params[2]);
		T theta = T(0.0);
		T min_distance = std::numeric_limits<T>::max();
		for (T t = T(0.0); t < T(2.0 * M_PI); t += T(0.01)) {
			Eigen::Matrix<T, 3, 1> p = c + params[3] * cos(t) * u_vec + params[4] * sin(t) * v;
			T distance = (point_.cast<T>() - p).squaredNorm();
			if (distance < min_distance) {
				min_distance = distance;
				theta = t;
			}
		}

		// ���������Ĳв�
		Eigen::Matrix<T, 3, 1> p = c + params[3] * cos(theta) * u_vec + params[4] * sin(theta) * v;
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
		// ���������ת��ΪEigen����
		Eigen::Matrix<T, 3, 1> center_(center[0], center[1], center[2]);
		Eigen::Matrix<T, 3, 1> normal_(normal[0], normal[1], normal[2]);
		Eigen::Matrix<T, 3, 1> u_axis_(u_axis[0], u_axis[1], u_axis[2]);

		// �㵽��Բ�ľ������
		Eigen::Matrix<T, 3, 1> diff = point_.cast<T>() - center_;

		// ��תu_axis��ʹ����diff����
		Eigen::Matrix<T, 3, 1> v_axis = normal_.cross(u_axis_).normalized();
		Eigen::Matrix<T, 3, 1> rotated_diff = Eigen::Matrix<T, 3, 1>(diff.dot(u_axis_), diff.dot(v_axis), diff.dot(normal_));

		// ��Բ����
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

		// ����㵽���ĵ�����
		Eigen::Matrix<T, 3, 1> diff = point_.cast<T>() - center;

		// ���㷨��������������
		Eigen::Matrix<T, 3, 1> v_axis = normal.cross(u_axis);    // .normalized();

		// ��תu_axis��ʹ����diff����
		Eigen::Matrix<T, 3, 1> rotated_diff(diff.dot(u_axis), diff.dot(v_axis), diff.dot(normal));  //ʹ��������������u�ᡢv��ͷ���������diff������ת��һ���µ�����ϵ����ʵ�������ҵ�diff����Բ��������ϵ�µ����꣩��

		// ��Բ����
		T ellipse_distance = (rotated_diff[0] * rotated_diff[0]) / (a * a) +
			(rotated_diff[1] * rotated_diff[1]) / (b * b);

		residuals[0] = ellipse_distance - T(1.0);

		return true;
	}

	static ceres::CostFunction* Create(const Eigen::Vector3d& point) {
		return (new ceres::AutoDiffCostFunction<EllipseResidualX, 1, 11>(
			new EllipseResidualX(point)));
	}

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

		// ���㷨��������������
		Eigen::Matrix<T, 3, 1> v = n.cross(u).normalized();

		// ����㵽���ĵ�����
		Eigen::Matrix<T, 3, 1> diff = point_.cast<T>() - c;

		// ��diff��ת����Բ�ı�������ϵ
		Eigen::Matrix<T, 3, 1> local_diff(u.dot(diff), v.dot(diff), n.dot(diff));

		// ���㵽��Բ����ľ���
		T local_distance = ceres::sqrt(local_diff[0] * local_diff[0] / (A * A) +
			local_diff[1] * local_diff[1] / (B * B));

		// �в�Ϊ�㵽��Բ������������
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

	// ʹ�� OpenMP ���л�ѭ��
#pragma omp parallel for
	for (int i = 0; i < cloud->points.size(); ++i)
	{
		vectors[i] = Eigen::Vector3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
	}

	return vectors;
}

int main()
{
	// -------------------------���ص���------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("ellipse1.pcd", *cloud) < 0)
	{
		PCL_ERROR("Couldn't read file \n");
		return -1;
	}
	// ------------------------RANSAC���-----------------------------   
	pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>::Ptr Ellipse3D(new pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(Ellipse3D);
	ransac.setDistanceThreshold(0.13);	        // ������ֵ����ģ�;���С��0.01�ĵ���Ϊ�ڵ�
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
	// �����Ͻ��
	std::cout << "Center: (" << params[0] << ", " << params[1] << ", " << params[2] << ")\n";
	std::cout << "Big diameter: " << params[3] * 2 << "\n";
	std::cout << "Small diameter: " << params[4] * 2 << "\n";
	std::cout << "Normal: (" << params[5] << ", " << params[6] << ", " << params[7] << ")\n";
	std::cout << "U axis: (" << params[8] << ", " << params[9] << ", " << params[10] << ")\n";

#if ENABLE_DISPLAY
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
#endif
	return 0;
}

