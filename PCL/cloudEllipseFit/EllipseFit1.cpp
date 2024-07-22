#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_ellipse3d.h>	// 拟合3D椭圆
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
	// 创建点云对象并填充数据（这是示例，实际使用时需要加载点云数据）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// ... 填充点云数据 ...

	// 创建RANSAC对象和平面模型对象
	pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelEllipse3D<pcl::PointXYZ>(cloud));
	const int num_iterations = 100; // 运行RANSAC的次数
	double best_rmse = std::numeric_limits<double>::max();
	std::vector<int> best_inliers;
	Eigen::VectorXf best_coefficients;

	for (int i = 0; i < num_iterations; ++i) {
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
		ransac.setDistanceThreshold(0.01); // 设置距离阈值
		ransac.computeModel();
		std::vector<int> inliers;
		ransac.getInliers(inliers);

		// 计算当前结果的RMSE
		double rmse = calculateRMSE(cloud, model, inliers);

		// 如果当前RMSE更小，则更新最优结果
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