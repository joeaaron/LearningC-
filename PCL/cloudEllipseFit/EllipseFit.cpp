#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_ellipse3d.h>	// ���3D��Բ
#include <iostream>
#include <cmath>
#include <limits>

double calculateRMSE(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
	const pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>::Ptr& model,
	const std::vector<int>& inliers) 
{
	Eigen::VectorXf coeff;
	//model->getModelCoefficients(coeff);
	double rmse = 0.0;

	for (const auto& idx : inliers)
	{
		pcl::PointXYZ point = cloud->points[idx];

	}
	rmse = std::sqrt(rmse / inliers.size());
	return rmse;
}

int main() {
	// �������ƶ���������ݣ�����ʾ����ʵ��ʹ��ʱ��Ҫ���ص������ݣ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// ... ���������� ...

	// ����RANSAC�����ƽ��ģ�Ͷ���
	pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>(cloud));
	const int num_iterations = 100; // ����RANSAC�Ĵ���
	double best_rmse = std::numeric_limits<double>::max();
	std::vector<int> best_inliers;
	Eigen::VectorXf best_coefficients;

	for (int i = 0; i < num_iterations; ++i) {
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
		ransac.setDistanceThreshold(0.01); // ���þ�����ֵ
		ransac.computeModel();
		std::vector<int> inliers;
		ransac.getInliers(inliers);

		// ���㵱ǰ�����RMSE
		double rmse = calculateRMSE(cloud, model, inliers);

		// �����ǰRMSE��С����������Ž��
		if (rmse < best_rmse) 
		{
			best_rmse = rmse;
			best_inliers = inliers;
			//model->getModelCoefficients(best_coefficients);
		}
	}

	std::cout << "Best RMSE: " << best_rmse << std::endl;
	std::cout << "Best Model Coefficients: " << best_coefficients.transpose() << std::endl;

	return 0;
}