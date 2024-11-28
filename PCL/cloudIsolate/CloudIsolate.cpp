#include <iostream>
#include <pcl/io/pcd_io.h>  
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>  

using namespace std;

#define ENABLE_DISPLAY 1				// ����һ���꣬���ڿ�����ʾ״̬

// ��������
struct PointCurvature
{
	int index;							// ����
	double curvature;					// ����
};

void PCLSorFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(2.6);				// ������Χ���ٸ��㣬�ҵ��㹻�������
	sor.setStddevMulThresh(1);
	sor.setNegative(true);
	sor.filter(*filteredCloud);
}

int main()
{
	//----------------------��ȡ����---------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("2.pcd", *pCloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	//----------------------��������---------------------
	vector<PointCurvature> vPointCurvature;
	double dRatio = 0.1;

	//----------------------����ִ��---------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr pFiltered(new pcl::PointCloud<pcl::PointXYZ>);
	PCLSorFilter(pFiltered, pCloud);

	//----------------------��ʾ���---------------------
#if ENABLE_DISPLAY
//---------------------------������ӻ�----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Cloud Filter"));

	// ���õ����ӿڱ�����ɫ
	MView->setBackgroundColor(0.3, 0.3, 0.3);

	// ���ԭʼ���Ʋ�����Ϊ����ɫ
	MView->addPointCloud<pcl::PointXYZ>(pCloud, "Raw cloud");
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.2627, 0.3451, 0.4588, "Raw cloud");

	// ��ӹ��˺�ĵ��Ʋ�����Ϊ��ɫ
	MView->addPointCloud<pcl::PointXYZ>(pFiltered, "Filtered cloud");
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Filtered cloud");

	// �������ϵ����ѡ��
	MView->addCoordinateSystem(1.0);

	// ��ʼ�������������������ӽ�
	MView->resetCamera();
	MView->initCameraParameters();

	// ��ʼ����ʽ���ӻ�
	MView->spin();
#endif
}