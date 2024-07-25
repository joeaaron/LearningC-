#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/pca.h>
#include <ceres/ceres.h>
#include "Line.h"

const int FitMethod = 2;
using namespace std;
#define ENABLE_DISPLAY 0		// ����һ���꣬���ڿ�����ʾ״̬

struct LineResidual 
{
    LineResidual(const Eigen::Vector3d& point) : point_(point) {}

	template <typename T>
	bool operator()(const T* const params, T* residual) const 
    {
		// Convert line parameters to Eigen::Matrix<T, 3, 1>
		Eigen::Matrix<T, 3, 1> line_p(params[0], params[1], params[2]);
		Eigen::Matrix<T, 3, 1> line_d(params[3], params[4], params[5]);

		// Normalize the direction vector
		line_d.normalize();

		// Calculate the residual as the distance from the point to the line
		Eigen::Matrix<T, 3, 1> point_to_line = point_ - line_p;
		residual[0] = (point_to_line - point_to_line.dot(line_d) * line_d).norm();

		return true;
	}
	static ceres::CostFunction* Create(const Eigen::Vector3d& point) {
		return (new ceres::AutoDiffCostFunction<LineResidual, 1, 6>(
			new LineResidual(point)));
	}
private:
    const Eigen::Vector3d point_;
};

void RANSAC_LineFit(pcl::PointCloud<pcl::PointXYZ>::Ptr& line, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    //---------------�������ģ��---------------------
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers�����洢ֱ���ϵ������
    pcl::SACSegmentation<pcl::PointXYZ> seg;//����һ���ָ���
    seg.setOptimizeCoefficients(true);      //��ѡ�����ã�����ģ��ϵ����Ҫ�Ż�
    seg.setModelType(pcl::SACMODEL_LINE);   //����Ŀ�꼸����״
    seg.setMethodType(pcl::SAC_RANSAC);     //��Ϸ��������������
    seg.setDistanceThreshold(0.05);         //����������̷�Χ��Ҳ������ֵ
    seg.setMaxIterations(500);              //�����������������õĻ�Ĭ�ϵ���50��
    seg.setInputCloud(cloud);               //�������
    seg.segment(*inliers, *coefficients);   //��ϵ���
    //------------------ģ��ϵ��---------------------
    cout << "���ֱ�ߵ�ģ��ϵ��Ϊ��" << endl;
    cout << "a��" << coefficients->values[0] << endl;
    cout << "b��" << coefficients->values[1] << endl;
    cout << "c��" << coefficients->values[2] << endl;
    cout << "d��" << coefficients->values[3] << endl;
    cout << "e��" << coefficients->values[4] << endl;
    cout << "f��" << coefficients->values[5] << endl;

    //--------------��ȡ��ϵ�ֱ��------------------
    /*ֱ����ȡ����1
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < inliers->indices.size(); ++i) {
        c_plane->points.push_back(cloud->points.at(inliers->indices[i]));
    }
    */
    //ֱ����ȡ
  
    pcl::ExtractIndices<pcl::PointXYZ> extract;  //����������ȡ����
    extract.setInputCloud(cloud);    //�����������
    extract.setIndices(inliers);     //���÷ָ����ڵ�Ϊ��Ҫ��ȡ�ĵ㼯
    extract.setNegative(false);      //false��ȡ�ڵ�, true��ȡ���
    extract.filter(*line);           //��ȡ����洢��c_plane2

	//-----------------���ƿ��ӻ�---------------
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud, "cloud");  // ���رȶԵ���
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> line_color(line, 255, 0, 0);
	viewer.addPointCloud(line, line_color, "line");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line");
	viewer.spin();
}

void LS_LineFit(pcl::PointCloud<pcl::PointXYZ>::Ptr& line, pcl::ModelCoefficients::Ptr& coeff, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);

    // ��ȡ����ֵ��Ӧ����������
    Eigen::RowVector3f v1 = pca.getEigenVectors().col(0);
	/* Eigen::RowVector3f v2 = pca.getEigenVectors().col(1);
	 Eigen::RowVector3f v3 = pca.getEigenVectors().col(2);*/
	 // У��ֱ�߷�����������֤z����ʼ��Ϊ�� 240725
	if (v1[2] < 0)
	{
		v1 = -v1;
	}
    // ֱ�ߵĵ���ʽ
    float m = v1[0], n = v1[1], p = v1[2];

    float x0 = pca.getMean()[0], y0 = pca.getMean()[1], z0 = pca.getMean()[2];
	coeff->values.push_back(x0);
	coeff->values.push_back(y0);
	coeff->values.push_back(z0);
	coeff->values.push_back(m);
	coeff->values.push_back(n);
	coeff->values.push_back(p);

#if ENABLE_DISPLAY
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud, "cloud");  // ���رȶԵ���
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> line_color(cloud, 255, 0, 0);
	viewer.addPointCloud(cloud, line_color, "line"); 
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line");
    viewer.setWindowName(u8"��С����ֱ�����");
    viewer.addLine(*coeff, "line");
    viewer.spin();
#endif
}

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
    //----------------��ȡ��������---------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("edge1.pcd", *cloud);    // L.pcd airplane.pcd
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr line(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    Eigen::Vector<float, 6> lineParam;

    if (1 == FitMethod)
    {
        RANSAC_LineFit(line, cloud);
    }
    else if (2 == FitMethod)
    {
        LS_LineFit(line, coeff, cloud);
    }  
    else
    {
		std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> points = cloud->points;
		LineFit<3, float, pcl::PointXYZ> lf;
		lf.setFittingPoints(points);
		lf.computeLineParam(lineParam);
		std::cout << "��ϲ�����" << lineParam.transpose() << std::endl;
    }

	double params[6] = {
          coeff->values[0], coeff->values[1], coeff->values[2],
          coeff->values[3], coeff->values[4], coeff->values[5]
	};
	/*double params[6] = { static_cast<double>(lineParam(0)), static_cast<double>(lineParam(1)),static_cast<double>(lineParam(2)),
    static_cast<double>(lineParam(3)), static_cast<double>(lineParam(4)),static_cast<double>(lineParam(5)) };*/

    // ceres�Ż�
    std::vector<Eigen::Vector3d> points = PointCloud2Vector3d(cloud);
	ceres::Problem problem;

	for (const auto& point : points)
	{
		ceres::CostFunction* cost_function = LineResidual::Create(point);
		problem.AddResidualBlock(cost_function, nullptr, params);	
	}

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	options.function_tolerance = 1e-6;					// �����ݲ�
	options.gradient_tolerance = 1e-6;					// �ݶ��ݲ�
	options.parameter_tolerance = 1e-6;					// �����ݲ�
	options.num_threads = 4;

	ceres::Solver::Summary summary;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	ceres::Solve(options, &problem, &summary);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
	cout << "Point size:" << cloud->points.size() << endl;
	cout << "Solve time cost = " << time_used.count() << " seconds. " << endl;

	std::cout << summary.BriefReport() << std::endl;

	// ��ȡֱ�߲���
	Eigen::Vector3d p(coeff->values[0], coeff->values[1], coeff->values[2]);
	Eigen::Vector3d d(coeff->values[3], coeff->values[4], coeff->values[5]);

	d.norm();		// ֱ�߷������������й�һ��

	std::vector<Eigen::Vector3d> projected_points;
	double min_t = std::numeric_limits<double>::max();
	double max_t = std::numeric_limits<double>::min();

	for (const auto& point : cloud->points) {
		Eigen::Vector3d q(point.x, point.y, point.z);
		double t = (q - p).dot(d) / d.dot(d);
		Eigen::Vector3d r = p + t * d;
		projected_points.push_back(r);

		if (t < min_t) min_t = t;
		if (t > max_t) max_t = t;
	}

	Eigen::Vector3d endpoint1 = p + min_t * d;
	Eigen::Vector3d endpoint2 = p + max_t * d;
	double length = (endpoint2 - endpoint1).norm();

	std::cout << "ԭ��: (" << endpoint1[0] << ", " << endpoint1[1] << ", " << endpoint1[2] << ")\n";
	std::cout << "����: (" << params[3] << ", " << params[4] << ", " << params[5] << ")\n";
	std::cout << "����:	" << length << "\n";

    return 0;
}


