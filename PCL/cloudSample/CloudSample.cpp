#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/common/common.h>
#include <pcl/console/time.h> 
#include <pcl/visualization/pcl_visualizer.h>

#define ENABLE_DISPLAY 0		// ����һ���꣬���ڿ�����ʾ״̬
const float KEEP_PERCENTAGE = 15;

// ����ṹ�����ڱ��������ÿ���������������
struct PointCurvature
{
	int index;
	float curvature;
};

// �������ʶԵ��������
bool SortFunction(const PointCurvature& a, const PointCurvature& b)
{
	return a.curvature < b.curvature;
}

// �������ʵĺ���
void ComputeCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
	const pcl::PointCloud<pcl::Normal>::Ptr& normals,
	std::vector<PointCurvature>& curvatureValues,
	size_t startIdx,
	size_t endIdx)
{
	for (size_t i = startIdx; i < endIdx; ++i)
	{
		curvatureValues[i].index = i;
		curvatureValues[i].curvature = normals->points[i].curvature;
	}
}

// ��������1
void CalCurvature1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// ���㷨��
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(10);		// ���÷��߼������������
	ne.compute(*cloud_normals);

	// ����洢���ʵĽṹ��
	std::vector<PointCurvature> curvatureValues(cloud->points.size());

	// �������ʣ����ö��̲߳��м���
#pragma omp parallel for num_threads(8)
	for (int i = 0; i < cloud->points.size(); ++i)
	{
		curvatureValues[i].index = i;
		curvatureValues[i].curvature = cloud_normals->points[i].curvature;
	}
	//pcl::parallel_for(0, cloud->points.size(), [&](size_t startIdx, size_t endIdx) {
	//	ComputeCurvature(cloud, cloud_normals, curvatureValues, startIdx, endIdx);
	//	});

	// ������������
	std::sort(curvatureValues.begin(), curvatureValues.end(), SortFunction);

	// ���㱣��������������ݰٷֱȲ�����
	size_t numPointsToKeep = static_cast<size_t>((KEEP_PERCENTAGE / 100.0) * curvatureValues.size());

	// ����ǰ numPointsToKeep ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
	sampledCloud->points.resize(numPointsToKeep);

#pragma omp parallel for num_threads(8)
	for (int i = 0; i < numPointsToKeep; ++i)
	{
		sampledCloud->points[i] = cloud->points[curvatureValues[i].index];
	}
	sampledCloud->width = numPointsToKeep;
	sampledCloud->height = 1;
	sampledCloud->is_dense = false;

#if ENABLE_DISPLAY
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Cloud curvature sampling"));

	int v1(0);
	MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
	MView->addText("Raw point clouds", 10, 10, "v1_text", v1);
	int v2(0);
	MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
	MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
	MView->addText("Sampled point clouds", 10, 10, "v2_text", v2);

	MView->addPointCloud<pcl::PointXYZ>(cloud, "Raw cloud", v1);
	MView->addPointCloud<pcl::PointXYZ>(sampledCloud, "Sampled cloud", v2);

	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Raw cloud", v1);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "Sampled cloud", v2);

	MView->addCoordinateSystem(1.0);
	MView->initCameraParameters();

	MView->spin();
#endif
	// ���������ĵ���
	pcl::io::savePCDFile("output.pcd", *sampledCloud);
}

// ��������2
void CalCurvature2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// ���㷨��
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(10);		// ���÷��߼������������
	ne.compute(*cloud_normals);

	// �������е������
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
	pc.setInputCloud(cloud);
	pc.setInputNormals(cloud_normals);
	pc.setSearchMethod(tree);

	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr pCurvature(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pc.setKSearch(10);
	pc.compute(*pCurvature);

	//������Ƶĸ�˹���ʣ���ȡ���ʼ�
	std::vector<PointCurvature> curvatureValues(cloud->points.size());;

#pragma omp parallel for num_threads(8)
	for (int i = 0; i < pCurvature->size(); i++) {
		//ƽ������
		//float curvature = (*pCurvature)[i].pc1;// +(*pCurvature)[i].pc2) / 2;
		//��˹����
		float curvature = (*pCurvature)[i].pc1 * (*pCurvature)[i].pc2;
		//pv.cPoint = tempPoint;
		curvatureValues[i].index = i;
		curvatureValues[i].curvature = curvature;
	}

	// ������������
	std::sort(curvatureValues.begin(), curvatureValues.end(), SortFunction);

	// ���㱣��������������ݰٷֱȲ�����
	size_t numPointsToKeep = static_cast<size_t>((KEEP_PERCENTAGE / 100.0) * curvatureValues.size());

	// ����ǰ numPointsToKeep ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
	sampledCloud->points.resize(numPointsToKeep);

#pragma omp parallel for num_threads(8)
	for (int i = 0; i < numPointsToKeep; ++i)
	{
		sampledCloud->points[i] = cloud->points[curvatureValues[i].index];
	}
	sampledCloud->width = numPointsToKeep;
	sampledCloud->height = 1;
	sampledCloud->is_dense = false;

#if ENABLE_DISPLAY
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Cloud curvature sampling"));

	int v1(0);
	MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
	MView->addText("Raw point clouds", 10, 10, "v1_text", v1);
	int v2(0);
	MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
	MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
	MView->addText("Sampled point clouds", 10, 10, "v2_text", v2);

	MView->addPointCloud<pcl::PointXYZ>(cloud, "Raw cloud", v1);
	MView->addPointCloud<pcl::PointXYZ>(sampledCloud, "Sampled cloud", v2);

	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Raw cloud", v1);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "Sampled cloud", v2);

	MView->addCoordinateSystem(1.0);
	MView->initCameraParameters();

	MView->spin();
#endif
}

int main()
{
	// ���ص�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud) == -1)		// sac_plane_test.pcd | Scan_0511_1713.pcd
	{
		PCL_ERROR("���ƶ�ȡʧ�� \n");
		return (-1);
	}

	pcl::console::TicToc time;
	time.tic();

	CalCurvature1(cloud);
	std::cout << "ֱ�Ӽ������ʲ�����ʱ�� " << time.toc() / 1000 << " ��" << std::endl;

	time.tic();
	CalCurvature2(cloud);
	std::cout << "��˹���ʲ�����ʱ�� " << time.toc() / 1000 << " ��" << std::endl;

	return 0;
}