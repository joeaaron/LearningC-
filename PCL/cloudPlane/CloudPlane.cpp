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

using namespace std;

void RANSCAPlaneFit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    //------------------------------------------RANSAC���--------------------------------------------------------   
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);//����RANSAC�㷨ģ��
    ransac.setDistanceThreshold(0.01);//�趨������ֵ
    ransac.setMaxIterations(100);     //��������������
    ransac.setProbability(0.99);      //���ô���Ⱥֵ��ѡ������һ����������������
    ransac.computeModel();            //���ƽ��
    vector<int> inliers;              //���ڴ���ڵ�������vector
    ransac.getInliers(inliers);       //��ȡ�ڵ�����

    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);  //��ȡ���ƽ�������coeff�ֱ�˳�򱣴�a,b,c,d

    cout << "ƽ��ģ��ϵ��coeff(a,b,c,d): " << coeff[0] << " \t" << coeff[1] << "\t " << coeff[2] << "\t " << coeff[3] << endl;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr sac_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *sac_plane);
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
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> after_sac(sac_plane, 0, 0, 255);

    viewer->addPointCloud(cloud, color, "cloud", v1);
    //viewer->addPointCloud(sac_plane, after_sac, "plane cloud", v2);

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

	cout << "ƽ��ģ��ϵ��Ϊ��\n"
		<< "A=" << normal[0] << "\n"
		<< "B=" << normal[1] << "\n"
		<< "C=" << normal[2] << "\n"
		<< "D=" << D << "\n" << endl;

	// ��ʾ��ϳ�����ƽ��
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
}

int
main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("sac_plane_test.pcd", *cloud) == -1)
	{
		PCL_ERROR("���ƶ�ȡʧ�� \n");
		return (-1);
	}

	//if (pcl::io::loadPLYFile<pcl::PointXYZ>("Scan_0511_1713.ply", *cloud) == -1)
	//{
	//	PCL_ERROR("���ƶ�ȡʧ�� \n");
	//	return (-1);
	//}
    //RANSCAPlaneFit(cloud);
    LeastSquareFit(cloud);

    return 0;
}

