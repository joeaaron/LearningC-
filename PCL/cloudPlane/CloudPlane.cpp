#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <chrono>

using namespace std;
#define ENABLE_DISPLAY 0  // 定义一个宏，用于控制显示状态

void RANSCAPlaneFit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    //------------------------------------------RANSAC框架--------------------------------------------------------   
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);//定义RANSAC算法模型
    ransac.setDistanceThreshold(0.01);//设定距离阈值
    ransac.setMaxIterations(100);     //设置最大迭代次数
    ransac.setProbability(0.99);      //设置从离群值中选择至少一个样本的期望概率
    ransac.computeModel();            //拟合平面
    vector<int> inliers;              //用于存放内点索引的vector
    ransac.getInliers(inliers);       //获取内点索引

    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);  //获取拟合平面参数，coeff分别按顺序保存a,b,c,d

    cout << "平面模型系数coeff(a,b,c,d): " << coeff[0] << " \t" << coeff[1] << "\t " << coeff[2] << "\t " << coeff[3] << endl;
    /*
     //-------------------平面法向量定向，与（1，1，1）同向，并输出平面与原点的距离D---------------------------
     double a, b, c, d, A, B, C, D;//a,b,c为拟合平面的单位法向量，A,B,C为重定向后的法向量
     a = coeff[0], b = coeff[1], c = coeff[2], d = coeff[3];

     if (a + b + c > 0) {
         A = a;
         B = b;
         C = c;
         D = abs(d);
     }
     else {
         A = -a;
         B = -b;
         C = -c;
         D = abs(d);
     }
     cout << "" << A << ",\t" << "" << B << ",\t" << "" << C << ",\t" << "" << D << ",\t" << endl;
     */

     //--------------------------------根据内点索引提取拟合的平面点云-----------------------------------
#if ENABLE_DISPLAY
	pcl::PointCloud<pcl::PointXYZ>::Ptr sac_plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *sac_plane);
	// pcl::io::savePCDFileASCII("1.11.pcd", *final);
	//-------------------------------------------可视化-------------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("cloud show"));
	int v1 = 0;
	int v2 = 1;

	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0, 0, 0, v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> after_sac(sac_plane, 0, 0, 255);

	viewer->addPointCloud(cloud, color, "cloud", v1);
	//viewer->addPointCloud(sac_plane, after_sac, "plane cloud", v2);

	// 显示拟合出来的平面
	pcl::ModelCoefficients plane;
	plane.values.push_back(coeff[0]);
	plane.values.push_back(coeff[1]);
	plane.values.push_back(coeff[2]);
	plane.values.push_back(coeff[3]);

	viewer->addPlane(plane, "cloud", v2);


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
#endif

}

void LeastSquareFit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	Eigen::Vector4d centroid;                    // 质心
	Eigen::Matrix3d covariance_matrix;           // 协方差矩阵

	// 计算归一化协方差矩阵和质心
	pcl::computeMeanAndCovarianceMatrix(*cloud, covariance_matrix, centroid);

	// 计算协方差矩阵的特征值与特征向量
	Eigen::Matrix3d eigenVectors;
	Eigen::Vector3d eigenValues;
	pcl::eigen33(covariance_matrix, eigenVectors, eigenValues);

	// 查找最小特征值的位置
	Eigen::Vector3d::Index minRow, minCol;
	eigenValues.minCoeff(&minRow, &minCol);

	// 获取平面方程：AX+BY+CZ+D = 0的系数
	Eigen::Vector3d normal = eigenVectors.col(minCol);
	double D = -normal.dot(centroid.head<3>());

	cout << "平面模型系数为：\n"
		<< "A=" << normal[0] << "\n"
		<< "B=" << normal[1] << "\n"
		<< "C=" << normal[2] << "\n"
		<< "D=" << D << "\n" << endl;

#if ENABLE_DISPLAY
	// 显示拟合出来的平面
	pcl::ModelCoefficients plane;
	plane.values.push_back(normal[0]);
	plane.values.push_back(normal[1]);
	plane.values.push_back(normal[2]);
	plane.values.push_back(D);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("cloud show"));
	int v1 = 0;

	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 0, 255, 0);
	viewer->addPlane(plane, "cloud", v1);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
#endif

}

int
main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("Scan_0511_1713.pcd", *cloud) == -1)		// sac_plane_test.pcd | Scan_0511_1713.pcd
	{
		PCL_ERROR("点云读取失败 \n");
		return (-1);
	}

	//if (pcl::io::loadPLYFile<pcl::PointXYZ>("Scan_0511_1713.ply", *cloud) == -1)
	//{
	//	PCL_ERROR("点云读取失败 \n");
	//	return (-1);
	//}
    auto startOp1 = std::chrono::high_resolution_clock::now();
    RANSCAPlaneFit(cloud);
    auto endOp1 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp1 = endOp1 - startOp1;
	std::cout << "Elapsed time for operation 1: " << elapsedOp1.count() << " seconds" << std::endl;

    auto startOp2 = std::chrono::high_resolution_clock::now();
    LeastSquareFit(cloud);
    auto endOp2 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp2 = endOp2 - startOp2;
	std::cout << "Elapsed time for operation 2: " << elapsedOp2.count() << " seconds" << std::endl;

    return 0;
}

