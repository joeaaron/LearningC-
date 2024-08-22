#include <vector>
#include <algorithm>
#include"PCARegistration.h"
using namespace std;

// 计算点云质心与特征向量
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
// PCA求解变换矩阵
Eigen::Matrix4f PCARegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr& P_cloud, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& X_cloud) 
{
	Eigen::Vector4f Cp;                  // P_cloud的质心
	Eigen::Matrix3f Up;                  // P_cloud的特征向量
	ComputeEigenVectorPCA(P_cloud, Cp, Up);// 计算P_cloud的质心和特征向量
	Eigen::Vector4f Cx;                  // X_cloud的质心
	Eigen::Matrix3f Ux;                  // X_cloud的特征向量
	ComputeEigenVectorPCA(X_cloud, Cx, Ux);// 计算X_cloud的质心和特征向量

	// 分别讨论主轴对应的8种情况，选择误差最小时对应的变换矩阵
	float error[8] = {};
	vector<Eigen::Matrix4f>MF;
	Eigen::Matrix4f final_RT = Eigen::Matrix4f::Identity();// 定义最终的变换矩阵
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
		R = (Up * Ux.inverse()).transpose(); // 计算旋转矩阵
		Eigen::Matrix<float, 3, 1> T;
		T = Cx.head<3>() - R * (Cp.head<3>());// 计算平移向量
		Eigen::Matrix4f RT = Eigen::Matrix4f::Identity();// 初始化齐次坐标4X4变换矩阵
		
		RT.block<3, 3>(0, 0) = R;// 构建4X4变换矩阵的旋转部分
		RT.block<3, 1>(0, 3) = T;// 构建4X4变换矩阵的平移部分
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr t_cloud(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::transformPointCloud(*P_cloud, *t_cloud, RT);

		// 计算每一种主轴对应的平均均方误差
		pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
		core.setInputSource(t_cloud);
		core.setInputTarget(X_cloud);
		
		boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);
		core.determineReciprocalCorrespondences(*cor);//双向K近邻搜索获取最近点对
		
		double mean = 0.0, stddev = 0.0;
		pcl::registration::getCorDistMeanStd(*cor, mean, stddev);
		error[nn] = mean;
		MF.push_back(RT);
	}

	// 获取误差最小时所对应的索引
	int min_index = distance(begin(error), min_element(error, error + 8));

	// 误差最小时对应的变换矩阵即为正确变换矩阵
	final_RT = MF[min_index];
	return final_RT;
}
// 可视化
void visualize_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& target, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& regist)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Registration"));
	int v1 = 0;
	int v2 = 1;
	viewer->setWindowName("PCA配准结果");
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.05, 0, 0, v2);
	viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
	viewer->addText("Registed point clouds", 10, 10, "v2_text", v2);
	
	//原始点云绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source, 0, 255, 0);
	//目标点云蓝色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target, 0, 0, 255);
	//转换后的源点云红色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transe(regist, 255, 0, 0);
	//viewer->setBackgroundColor(255, 255, 255);
	viewer->addPointCloud(source, src_h, "source cloud", v1);
	viewer->addPointCloud(target, tgt_h, "target cloud", v1);
	viewer->addPointCloud(target, tgt_h, "target cloud1", v2);
	viewer->addPointCloud(regist, transe, "pcs cloud", v2);
	//添加坐标系
	//viewer->addCoordinateSystem(0.1);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
}
