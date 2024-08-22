#include <vector>
#include <algorithm>
#include"PCARegistration.h"
using namespace std;

// ���������������������
void ComputeEigenVectorPCA(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
	Eigen::Vector4f& pcaCentroid, 
	Eigen::Matrix3f& eigenVectorsPCA)
{
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	eigenVectorsPCA = eigen_solver.eigenvectors();
}
// PCA���任����
Eigen::Matrix4f PCARegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr& P_cloud, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& X_cloud) 
{
	Eigen::Vector4f Cp;                  // P_cloud������
	Eigen::Matrix3f Up;                  // P_cloud����������
	ComputeEigenVectorPCA(P_cloud, Cp, Up);// ����P_cloud�����ĺ���������
	Eigen::Vector4f Cx;                  // X_cloud������
	Eigen::Matrix3f Ux;                  // X_cloud����������
	ComputeEigenVectorPCA(X_cloud, Cx, Ux);// ����X_cloud�����ĺ���������

	// �ֱ����������Ӧ��8�������ѡ�������Сʱ��Ӧ�ı任����
	float error[8] = {};
	vector<Eigen::Matrix4f>MF;
	Eigen::Matrix4f final_RT = Eigen::Matrix4f::Identity();// �������յı任����
	Eigen::Matrix3f Upcopy = Up;

	int sign1[8] = { 1, -1,1,1,-1,-1,1,-1 };
	int sign2[8] = { 1, 1,-1,1,-1,1,-1,-1 };
	int sign3[8] = { 1, 1, 1,-1,1,-1,-1,-1 };

	for (int nn = 0; nn < 8; nn++)
	{
		Up.col(0) = sign1[nn] * Upcopy.col(0);
		Up.col(1) = sign2[nn] * Upcopy.col(1);
		Up.col(2) = sign3[nn] * Upcopy.col(2);

		Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
		R = (Up * Ux.inverse()).transpose(); // ������ת����
		Eigen::Matrix<float, 3, 1> T;
		T = Cx.head<3>() - R * (Cp.head<3>());// ����ƽ������
		Eigen::Matrix4f RT = Eigen::Matrix4f::Identity();// ��ʼ���������4X4�任����
		
		RT.block<3, 3>(0, 0) = R;// ����4X4�任�������ת����
		RT.block<3, 1>(0, 3) = T;// ����4X4�任�����ƽ�Ʋ���
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::transformPointCloud(*P_cloud, *t_cloud, RT);

		// ����ÿһ�������Ӧ��ƽ���������
		pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
		core.setInputSource(t_cloud);
		core.setInputTarget(X_cloud);
		
		boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);
		core.determineReciprocalCorrespondences(*cor);//˫��K����������ȡ������
		
		double mean = 0.0, stddev = 0.0;
		pcl::registration::getCorDistMeanStd(*cor, mean, stddev);
		error[nn] = mean;
		MF.push_back(RT);
	}

	// ��ȡ�����Сʱ����Ӧ������
	int min_index = distance(begin(error), min_element(error, error + 8));

	// �����Сʱ��Ӧ�ı任����Ϊ��ȷ�任����
	final_RT = MF[min_index];
	return final_RT;
}
// ���ӻ�
void visualize_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& target, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& regist)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Registration"));
	int v1 = 0;
	int v2 = 1;
	viewer->setWindowName("PCA��׼���");
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.05, 0, 0, v2);
	viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
	viewer->addText("Registed point clouds", 10, 10, "v2_text", v2);
	
	//ԭʼ������ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source, 0, 255, 0);
	//Ŀ�������ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 0, 0, 255);
	//ת�����Դ���ƺ�ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transe(regist, 255, 0, 0);
	//viewer->setBackgroundColor(255, 255, 255);
	viewer->addPointCloud(source, src_h, "source cloud", v1);
	viewer->addPointCloud(target, tgt_h, "target cloud", v1);
	viewer->addPointCloud(target, tgt_h, "target cloud1", v2);
	viewer->addPointCloud(regist, transe, "pcs cloud", v2);
	//�������ϵ
	//viewer->addCoordinateSystem(0.1);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
}
