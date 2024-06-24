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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <chrono>

#define ENABLE_DISPLAY	  1					// ����һ���꣬���ڿ�����ʾ״̬
#define ENABLE_PLANE_INFO 1

using namespace std;

void SEGPlaneFit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::ModelCoefficients::Ptr coefficients_m(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);							  // ��������������
	seg.setDistanceThreshold(0.01);						  // �趨��ֵ
	//seg.setNumberOfThreads(10);                           // �����߳�����
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients_m);

	Eigen::VectorXf coefficients;
	coefficients.resize(4);
	coefficients[0] = coefficients_m->values[0]; coefficients[1] = coefficients_m->values[1];
	coefficients[2] = coefficients_m->values[2]; coefficients[3] = coefficients_m->values[3];

#if ENABLE_PLANE_INFO
	cout << "ƽ��ģ��ϵ��Ϊ��\n"
		<< "A=" << coefficients[0] << "\n"
		<< "B=" << coefficients[1] << "\n"
		<< "C=" << coefficients[2] << "\n"
		<< "D=" << coefficients[3] << "\n" << endl;
#endif

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers); 
	extract.setNegative(true);
	pcl::PointCloud<pcl::PointXYZ>::Ptr segCloud(new pcl::PointCloud<pcl::PointXYZ>);
	extract.filter(*segCloud);
	 //--------------------------------�����ڵ�������ȡ��ϵ�ƽ�����-----------------------------------
#if ENABLE_DISPLAY
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr sac_plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *sac_plane);*/
	// pcl::io::savePCDFileASCII("1.11.pcd", *final);
	//-------------------------------------------���ӻ�-------------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("cloud show"));
	int v1 = 0;
	int v2 = 1;

	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0, 0, 0, v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> after_sac(segCloud, 0, 0, 255);

	viewer->addPointCloud(cloud, color, "cloud", v1);
	viewer->addPointCloud(segCloud, after_sac, "plane cloud", v2);

	// ��ʾ��ϳ�����ƽ��
	pcl::ModelCoefficients plane;
	plane.values.push_back(coefficients[0]);
	plane.values.push_back(coefficients[1]);
	plane.values.push_back(coefficients[2]);
	plane.values.push_back(coefficients[3]);

	viewer->addPlane(plane, "cloud", v2);


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
#endif

}

void RANSCAPlaneFit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    //------------------------------------------RANSAC���--------------------------------------------------------   
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);//����RANSAC�㷨ģ��
    ransac.setDistanceThreshold(0.01);//�趨������ֵ
    ransac.setMaxIterations(100);     //��������������
    ransac.setProbability(0.99);      //���ô���Ⱥֵ��ѡ������һ����������������
	//ransac.setNumberOfThreads(8);	  //���̼߳���
    ransac.computeModel();            //���ƽ��
    vector<int> inliers;              //���ڴ���ڵ�������vector
    ransac.getInliers(inliers);       //��ȡ�ڵ�����

    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);  //��ȡ���ƽ�������coeff�ֱ�˳�򱣴�a,b,c,d
	Eigen::VectorXf coefficients;
	model_plane->optimizeModelCoefficients(inliers, coeff, coefficients);      // �Ż�ƽ�淽�̲���

#if ENABLE_PLANE_INFO
	cout << "ƽ��ģ��ϵ��Ϊ��\n"
	<< "A=" << coefficients[0] << "\n"
	<< "B=" << coefficients[1] << "\n"
	<< "C=" << coefficients[2] << "\n"
	<< "D=" << coefficients[3] << "\n" << endl;
#endif

	
	//std::vector<int> remainIndices;
	//for (int i = 0; i < cloud->size(); ++i)
	//{
	//	if (std::find(inliers.begin(), inliers.end(), i) == inliers.end())
	//	{
	//		remainIndices.emplace_back(i);
	//	}
	//}
	//pcl::PointCloud<pcl::PointXYZ>::Ptr segCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//for (int i = 0; i < cloud->size(); ++i)
	//{
	//	Eigen::Vector3d point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
	//	Eigen::Vector3d normal(coefficients[0], coefficients[1], coefficients[2]);
	//	if ((point.dot(normal) + coefficients[3]) > 0)
	//	{
	//		segCloud->points.emplace_back(cloud->points[i]);
	//	}
	//}

    /*
     //-------------------ƽ�淨���������루1��1��1��ͬ�򣬲����ƽ����ԭ��ľ���D---------------------------
     double a, b, c, d, A, B, C, D;//a,b,cΪ���ƽ��ĵ�λ��������A,B,CΪ�ض����ķ�����
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

     //--------------------------------�����ڵ�������ȡ��ϵ�ƽ�����-----------------------------------
#if ENABLE_DISPLAY
	pcl::PointCloud<pcl::PointXYZ>::Ptr sac_plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *sac_plane);
	// pcl::io::savePCDFileASCII("1.11.pcd", *final);
	//-------------------------------------------���ӻ�-------------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud plane fit"));

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v1);
	viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
	viewer->setBackgroundColor(0.5, 0.5, 0.5, v2);
	viewer->addText("Plane fit cloud", 10, 10, "v2_text", v2);

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "Raw cloud", v1);
	viewer->addPointCloud<pcl::PointXYZ>(sac_plane, "Plane fit cloud", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Raw cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "Plane fit cloud", v2);

	//viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	// ��ʾ��ϳ�����ƽ��
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
	Eigen::Vector4d centroid;                    // ����
	Eigen::Matrix3d covariance_matrix;           // Э�������

	// �����һ��Э������������
	pcl::computeMeanAndCovarianceMatrix(*cloud, covariance_matrix, centroid);

	// ����Э������������ֵ����������
	Eigen::Matrix3d eigenVectors;
	Eigen::Vector3d eigenValues;
	pcl::eigen33(covariance_matrix, eigenVectors, eigenValues);

	// ������С����ֵ��λ��
	Eigen::Vector3d::Index minRow, minCol;
	eigenValues.minCoeff(&minRow, &minCol);

	// ��ȡƽ�淽�̣�AX+BY+CZ+D = 0��ϵ��
	Eigen::Vector3d normal = eigenVectors.col(minCol);
	double D = -normal.dot(centroid.head<3>());

#if ENABLE_PLANE_INFO
	cout << "ƽ��ģ��ϵ��Ϊ��\n"
		<< "A=" << normal[0] << "\n"
		<< "B=" << normal[1] << "\n"
		<< "C=" << normal[2] << "\n"
		<< "D=" << D << "\n" << endl;
#endif

	pcl::PointCloud<pcl::PointXYZ>::Ptr segCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < cloud->size(); ++i)
	{
		Eigen::Vector3d point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		if ((point.dot(normal) + D) > 0)
		{
			segCloud->points.emplace_back(cloud->points[i]);
		}
		/*double dRes = cloud->points[i].x * normal[0] + cloud->points[i].y * normal[1] + cloud->points[i].z * normal[2] + D;
		if (dRes > 0)
		{
			segCloud->points.emplace_back(cloud->points[i]);
		}*/
	}
#if ENABLE_DISPLAY
	// ��ʾ��ϳ�����ƽ��
	pcl::ModelCoefficients plane;
	plane.values.push_back(normal[0]);
	plane.values.push_back(normal[1]);
	plane.values.push_back(normal[2]);
	plane.values.push_back(D);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud plane fit"));

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v1);
	viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
	viewer->setBackgroundColor(0.5, 0.5, 0.5, v2);
	viewer->addText("Plane fit cloud", 10, 10, "v2_text", v2);

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "Raw cloud", v1);
	viewer->addPointCloud<pcl::PointXYZ>(segCloud, "Plane fit cloud", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Raw cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "Plane fit cloud", v2);

	//viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	// ��ʾ��ϳ�����ƽ��
	//viewer->addPlane(plane, "cloud", v2);

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

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud) == -1)		// sac_plane_test.pcd | Scan_0511_1713.pcd
	{
		PCL_ERROR("���ƶ�ȡʧ�� \n");
		return (-1);
	}

	//if (pcl::io::loadPLYFile<pcl::PointXYZ>("Scan_0511_1713.ply", *cloud) == -1)
	//{
	//	PCL_ERROR("���ƶ�ȡʧ�� \n");
	//	return (-1);
	//}
	auto startOp1 = std::chrono::high_resolution_clock::now();
	RANSCAPlaneFit(cloud);
	auto endOp1 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp1 = endOp1 - startOp1;
	std::cout << "RANSACƽ�����: " << elapsedOp1.count() << " seconds" << std::endl;

	auto startOp2 = std::chrono::high_resolution_clock::now();
	SEGPlaneFit(cloud);
	auto endOp2 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp2 = endOp2 - startOp2;
	std::cout << "RANSAC�ָ����: " << elapsedOp2.count() << " seconds" << std::endl;

	auto startOp3 = std::chrono::high_resolution_clock::now();
	LeastSquareFit(cloud);
	auto endOp3 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp3 = endOp3 - startOp3;
	std::cout << "��С����ƽ�����: " << elapsedOp3.count() << " seconds" << std::endl;

    return 0;
}

