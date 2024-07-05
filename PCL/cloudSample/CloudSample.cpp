#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/console/print.h>
#include <pcl/common/common.h>
#include <pcl/console/time.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#define ENABLE_DISPLAY 1					// ����һ���꣬���ڿ�����ʾ״̬
#define ENABLE_TEST 0						// ����һ���꣬���ڿ����㷨����

const float KEEP_PERCENTAGE = 15;
const int TESTS_NUM = 10;

using namespace pcl::io;
using namespace pcl::console;

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

// ���ʲ�������ʾ
void CurvatureSample(std::vector<PointCurvature>& curvatureValues, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
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

	//MView->addCoordinateSystem(1.0);
	MView->initCameraParameters();

	MView->spin();
#endif
	// ���������ĵ���
	pcl::io::savePCDFile("output.pcd", *sampledCloud);
}

// ��������1
void CalCurvature1(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	// ���㷨��
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(10);				// ���÷��߼������������
	ne.setNumberOfThreads(4);
	ne.compute(*cloudNormals);

	// ����洢���ʵĽṹ��
	std::vector<PointCurvature> curvatureValues(cloud->points.size());

	// �������ʣ����ö��̲߳��м���
#pragma omp parallel for num_threads(4)
	for (int i = 0; i < cloud->points.size(); ++i)
	{
		curvatureValues[i].index = i;
		curvatureValues[i].curvature = cloudNormals->points[i].curvature;
	}

	CurvatureSample(curvatureValues, cloud);
}

// ��������2
void CalCurvature2(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	// ���㷨��
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(10);		// ���÷��߼������������
	unsigned int num_threads = std::thread::hardware_concurrency();
	ne.setNumberOfThreads(num_threads);
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

	CurvatureSample(curvatureValues, cloud);
}

// ���㷨�ߺ�����
void computeNormalsAndCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
	pcl::PointCloud<pcl::Normal>::Ptr& normals,
	std::vector<PointCurvature>& curvatureValues,
	std::mutex& mtx,
	size_t startIdx,
	size_t endIdx)
{
	// �������߹�����
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	// ���÷��߼�������������������뾶
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	ne.setKSearch(10);		// ���÷��߼������������

	// ���㷨��
	normals.reset(new pcl::PointCloud<pcl::Normal>);
	ne.compute(*normals);

	// �������ʲ��洢���
	for (size_t i = startIdx; i < endIdx; ++i)
	{
		std::lock_guard<std::mutex> lk(mtx);

		curvatureValues[i].index = i;
		curvatureValues[i].curvature = normals->points[i].curvature;
	}
}

// ��������3
void CalCurvature3(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{

	// ����洢���ߺ����ʵ�����
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	std::vector<PointCurvature> curvatureValues(cloud->size());

	// ʹ�û����������������� curvatureValues
	std::mutex mtx;

	// ��ȡ���õ� CPU ������
	unsigned int num_threads = std::thread::hardware_concurrency();

//	// ���м��㷨�ߺ�����
//#pragma omp parallel sections
//	{
//#pragma omp section
//		{
//			// ���㷨��
//			pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
//			ne.setInputCloud(cloud);
//			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//			ne.setSearchMethod(tree);
//			ne.setKSearch(10);
//			ne.compute(*normals);
//
//			// ����洢���ʵĽṹ��
//			std::vector<PointCurvature> curvatureValues(cloud->points.size());
//			for (int i = 0; i < cloud->points.size(); ++i)
//			{
//				curvatureValues[i].index = i;
//				curvatureValues[i].curvature = normals->points[i].curvature;
//			}
//		}
//	}
	std::vector<std::thread> threads(num_threads);
	size_t chunk_size = cloud->size() / num_threads;
	size_t startIdx = 0;
	for (unsigned int i = 0; i < num_threads; ++i)
	{
		size_t endIdx = (i == num_threads - 1) ? cloud->size() : startIdx + chunk_size;
		threads[i] = std::thread([&cloud, &normals, &curvatureValues, &mtx, startIdx, endIdx]() {
			computeNormalsAndCurvature(cloud, normals, curvatureValues, mtx, startIdx, endIdx);
			});
		startIdx = endIdx;
	}

	// �ȴ������߳̽���
	for (auto& t : threads)
	{
		t.join();
	}

	CurvatureSample(curvatureValues, cloud);
}

void PCLVoxelSample(pcl::PointCloud<pcl::PointXYZ>::Ptr& sampledCloud, 
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(1, 1, 1);
	vg.filter(*sampledCloud);
}

/**
 * @description:			�����˲�
 * @param cloud				�������
 * @param cloud_filtered	�˲�����
 * @param leafsize			���ش�С
 */
void VoxelSample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudSampled, 
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudIn, float leafsize)
{
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*cloudIn, minPt, maxPt);

	float lx = maxPt.x - minPt.x;
	float ly = maxPt.y - minPt.y;
	float lz = maxPt.z - minPt.z;

	int nx = static_cast<int>(std::ceil(lx / leafsize));
	int ny = static_cast<int>(std::ceil(ly / leafsize));
	int nz = static_cast<int>(std::ceil(lz / leafsize));

	std::vector<pcl::PointCloud<pcl::PointXYZ>> v(nx * ny * nz);

	for (int i = 0; i < cloudIn->points.size(); i++)
	{
		int ix = static_cast<int>(std::floor((cloudIn->points[i].x - minPt.x) / leafsize));
		int iy = static_cast<int>(std::floor((cloudIn->points[i].y - minPt.y) / leafsize));
		int iz = static_cast<int>(std::floor((cloudIn->points[i].z - minPt.z) / leafsize));

		// ע���������������
		v[ix + iy * nx + iz * (nx * ny)].push_back(cloudIn->points[i]);
	}

	for (int i = 0; i < v.size(); ++i)
	{
		if (!v[i].empty())
		{
			Eigen::Vector4f centroid;
			pcl::compute3DCentroid(v[i], centroid);
			cloudSampled->push_back(pcl::PointXYZ(centroid.x(), centroid.y(), centroid.z()));
		}
	}
}

int main()
{
	// ���ص�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("1600w.pcd", *cloud) == -1)		//
	{
		PCL_ERROR("���ƶ�ȡʧ�� \n");
		return (-1);
	}

	pcl::console::TicToc tt;
	//time.tic();
	//CalCurvature1(cloud);
	//std::cout << "ֱ�����ʲ�����ʱ�� " << time.toc() / 1000 << " ��" << std::endl;
	//
	//time.tic();
	//CalCurvature2(cloud);
	//std::cout << "��˹���ʲ�����ʱ�� " << time.toc() / 1000 << " ��" << std::endl;
	//
	//time.tic();
	//CalCurvature3(cloud);
	//std::cout << "���߳�ֱ�����ʲ�����ʱ�� " << time.toc() / 1000 << " ��" << std::endl;
	//
	
	print_highlight("Sampling: "); 
	print_value("%s ", "table_scene_lms400.pcd");
	tt.tic();
	pcl::PointCloud<pcl::PointXYZ>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
	VoxelSample(sampledCloud, cloud, 1);
	//PCLVoxelSample(sampledCloud, cloud);
	//std::cout << "���ز�����ʱ�� " << time.toc() / 1000 << " ��" << std::endl;
	print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms: "); print_value("%d", sampledCloud->width * sampledCloud->height); print_info(" points]\n");

#if ENABLE_TEST
	// �㷨���Կ��
	std::vector<double> vTimes;
	double dMaxTime = 0.0;

	for (int i = 0; i < TESTS_NUM; ++i)
	{
		auto start = std::chrono::high_resolution_clock::now();
		CalCurvature1(cloud);
		auto end = std::chrono::high_resolution_clock::now();

		std::chrono::duration<double, std::milli> duration = end - start;
		vTimes.emplace_back(duration.count());

		if (duration.count() > dMaxTime)
		{
			dMaxTime = duration.count();
		}
	}
	
	double dAverageTime = std::accumulate(vTimes.begin(), vTimes.end(), 0.0) / vTimes.size();
	std::cout << "ֱ�����ʲ���ƽ����ʱ�� " << dAverageTime << "ms" << std::endl;
	std::cout << "ֱ�����ʲ��������ʱ�� " << dMaxTime << " ms" << std::endl;

	//std::cout << "��˹���ʲ���ƽ����ʱ�� " << dAverageTime << "ms" << std::endl;
	//std::cout << "��˹���ʲ��������ʱ�� " << dMaxTime << " ms" << std::endl;
#endif	

#if ENABLE_DISPLAY
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Cloud Sample"));

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

	MView->initCameraParameters();
	MView->spin();
#endif

	return 0;
}