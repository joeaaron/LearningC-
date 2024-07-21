#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_ellipse3d.h>// ���3D��Բ
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

int main()
{
	// -------------------------���ص���------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("ellipse.pcd", *cloud) < 0)
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

