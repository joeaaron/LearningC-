#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/boundary.h>
#include <math.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/boundary.h>
#include <pcl/io/ply_io.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/console/time.h> 
#include <pcl/surface/concave_hull.h>  
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#define ENABLE_DISPLAY 1		// ����һ���꣬���ڿ�����ʾ״̬
#define ENABLE_TEST 0			// ����һ���꣬���ڿ����㷨����

const int TESTS_NUM = 20;
const int KSERACH_SIZE = 70;

namespace {
	void ComputeNormals(pcl::PointCloud<pcl::Normal>::Ptr& pNormal, const pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudPtr)
	{
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;

		ne.setInputCloud(pCloudPtr);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		ne.setSearchMethod(kdtree);
		ne.setKSearch(KSERACH_SIZE);
		ne.setNumberOfThreads(4);
		ne.compute(*pNormal);
	}

	// �ڵ�ǰ�㽨������ϵ
	//			  �ֱ�Ϊ������n��������z�ᣨ��x�ᣩ��˵õ�������u���Լ�����n��u�Ĳ�˵õ�������v
	// �����ֲ���������ϵ��n, u, v���Ѿ�����λ������
	void GetCoordinateSystemOnPlane(const pcl::Normal& p_coeff, Eigen::Vector4f& u, Eigen::Vector4f& v)
	{
		pcl::Vector4fMapConst p_coeff_v = p_coeff.getNormalVector4fMap();
		v = p_coeff_v.unitOrthogonal();
		u = p_coeff_v.cross3(v);
	}

	bool IsBoundaryPoint(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointXYZ& q_point,
		const std::vector<int>& indices,
		const Eigen::Vector4f& u, const Eigen::Vector4f& v,
		const float angle_threshold)
	{
		if (indices.size() < 3) return false;
		if (!std::isfinite(q_point.x) || !std::isfinite(q_point.y) || !std::isfinite(q_point.z)) return false;

		std::vector<float> angles(indices.size());
		float max_dif = FLT_MIN, dif;
		int cp = 0;

		for (const auto& index : indices)
		{
			if (!std::isfinite(cloud->points[index].x) ||
				!std::isfinite(cloud->points[index].y) ||
				!std::isfinite(cloud->points[index].z)) continue;

			Eigen::Vector4f delta = cloud->points[index].getVector4fMap() - q_point.getVector4fMap();

			if (delta == Eigen::Vector4f::Zero()) continue;
			angles[cp++] = std::atan2(v.dot(delta), u.dot(delta));
		}

		if (cp == 0) return false;
		angles.resize(cp);
		std::sort(angles.begin(), angles.end());

		for (int i = 0; i < angles.size() - 1; i++)
		{
			dif = angles[i + 1] - angles[i];
			max_dif = std::max(max_dif, dif);
		}

		dif = 2 * static_cast<float> (M_PI) - angles.back() + angles[0];
		max_dif = std::max(max_dif, dif);
		return (max_dif > angle_threshold);
	}
}

void CloudBoundaryHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::console::TicToc time;
	time.tic();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConcaveHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud); // �������ΪͶӰ��ĵ���
	chull.setAlpha(1.2);        // ����alphaֵΪ0.1
	chull.reconstruct(*cloud_hull);
	
#if ENABLE_DISPLAY
	cout << "AS��ȡ�߽�����Ϊ: " << cloud_hull->points.size() << endl;
	cout << "AS��ȡ�߽����ʱ: " << time.toc() / 1000 << " ��" << endl;

	pcl::PCDWriter writer;
	writer.write("hull.pcd", *cloud_hull, false);

	//-----------------�����ʾ---------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud boundary extraction - AS"));

	int v1(0), v2(0);
	viewer->setWindowName("Cloud boundary extraction - AS");
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	viewer->addPointCloud(cloud_hull, "cloud_boundary");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_boundary");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_boundary");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(1000);
	}

#endif
}

int CloudBoundaryACEX(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::console::TicToc time;
	time.tic();

	// ����������
	pcl::PointCloud<pcl::Normal>::Ptr pNormal(new pcl::PointCloud<pcl::Normal>);
	ComputeNormals(pNormal, cloud);

	// ����kdtree
	pcl::search::KdTree<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	pcl::PointCloud<pcl::Boundary>::Ptr outBoundary(new pcl::PointCloud<pcl::Boundary>);
	outBoundary->resize(cloud->size());

#pragma omp parallel for
	for (int i = 0; i < cloud->size(); i++)
	{
		std::vector<int> vNborIdx;
		std::vector<float> vNborDist;
		kdtree.nearestKSearch(cloud->points[i], KSERACH_SIZE, vNborIdx, vNborDist);

		Eigen::Vector4f u = Eigen::Vector4f::Zero(), v = Eigen::Vector4f::Zero();
		GetCoordinateSystemOnPlane(pNormal->points[i], u, v);

		bool isBoundary = IsBoundaryPoint(cloud, cloud->points[i], vNborIdx, u, v, M_PI * 0.6);

		// ʹ�û����������ٽ�����
#pragma omp critical
		{
			outBoundary->points[i].boundary_point = isBoundary;
		}
	}

//	std::vector<int> vNborIdx;
//	std::vector<float> vNborDist;
//
//	// �߽���
//#pragma omp parallel for reduction(:vNborIdx)
//	for (int i = 0; i < cloud->size(); i++)
//	{
//		kdtree.nearestKSearch(cloud->points[i], 30, vNborIdx, vNborDist);
//		GetCoordinateSystemOnPlane(pNormal->points[i], u, v);
//		outBoundary->points[i].boundary_point = IsBoundaryPoint(cloud, cloud->points[i], vNborIdx, u, v, M_PI * 0.6);
//	}

	// ת��Ϊ�߽������
	std::vector<int> vIdx;
	//vIndex.clear();
	for (int i = 0; i < outBoundary->size(); ++i)
	{
		if (outBoundary->points[i].boundary_point != 0)
		{
			vIdx.emplace_back(i);
		}
	}

	cout << "�Ż���AC���߽����ʱ�� " << time.toc() << "ms" << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, vIdx, *cloud_boundary);
	cout << "�Ż���AC���߽�����Ϊ�� " << cloud_boundary->size() << endl;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZRGB>);
	//cloud_visual->resize(cloud->size());
	//for (size_t i = 0; i < cloud->size(); i++)
	//{
	//	cloud_visual->points[i].x = cloud->points[i].x;
	//	cloud_visual->points[i].y = cloud->points[i].y;
	//	cloud_visual->points[i].z = cloud->points[i].z;
	//	if (outBoundary->points[i].boundary_point != 0)
	//	{
	//		cloud_visual->points[i].r = 255;
	//		cloud_visual->points[i].g = 0;
	//		cloud_visual->points[i].b = 0;
	//		cloud_boundary->push_back(cloud_visual->points[i]);
	//	}
	//	else
	//	{
	//		cloud_visual->points[i].r = 255;
	//		cloud_visual->points[i].g = 255;
	//		cloud_visual->points[i].b = 255;
	//	}
	//}
#if ENABLE_DISPLAY
	//-----------------�����ʾ---------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud boundary extraction - AS"));

	int v1(0), v2(0);
	viewer->setWindowName("Cloud boundary extraction - ACEX");
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	viewer->addPointCloud(cloud_boundary, "cloud_boundary");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_boundary");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_boundary");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(1000);
	}
#endif
	return 0;
}

int CloudBoundaryAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::console::TicToc time;
	time.tic();

	// 1 ���㷨����
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr  normals(new  pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setNumberOfThreads(4);

	ne.setKSearch(20);								// ����
	//normalEstimation.setRadiusSearch(0.02);		// �뾶
	ne.compute(*normals);

	/*pcl����߽�*/
	pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>); //����һ��boundary��ָ�룬��Ϊ����ֵ
	boundaries->resize(cloud->size());				// ��ʼ����С

	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> be; //����һ��BoundaryEstimation��
	be.setInputCloud(cloud);						// �����������
	be.setInputNormals(normals);					// �������뷨��

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ptr(new pcl::search::KdTree<pcl::PointXYZ>);
	be.setSearchMethod(kdtree_ptr);					// ������Ѱk���ڵķ�ʽ
	be.setKSearch(30);								// ����k��������
	be.setAngleThreshold(M_PI * 0.6);				// ���ýǶ���ֵ��������ֵΪ�߽�
	be.compute(*boundaries);						// ������Ʊ߽磬���������boundaries��

	cout << "AC���߽����ʱ�� " << time.toc() / 1000 << " ��" << endl;

#if ENABLE_DISPLAY
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_visual->resize(cloud->size());
	for (size_t i = 0; i < cloud->size(); i++)
	{
		cloud_visual->points[i].x = cloud->points[i].x;
		cloud_visual->points[i].y = cloud->points[i].y;
		cloud_visual->points[i].z = cloud->points[i].z;
		if (boundaries->points[i].boundary_point != 0)
		{
			cloud_visual->points[i].r = 255;
			cloud_visual->points[i].g = 0;
			cloud_visual->points[i].b = 0;
			cloud_boundary->push_back(cloud_visual->points[i]);
		}
		else
		{
			cloud_visual->points[i].r = 255;
			cloud_visual->points[i].g = 255;
			cloud_visual->points[i].b = 255;
		}
	}
	cout << "AC���߽�����Ϊ   ��  " << cloud_boundary->size() << endl;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Cloud boundary extraction - AC"));

	int v1(0);
	MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
	MView->addText("Raw point clouds", 10, 10, "v1_text", v1);
	int v2(0);
	MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
	MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
	MView->addText("Boudary point clouds", 10, 10, "v2_text", v2);

	MView->addPointCloud<pcl::PointXYZRGB>(cloud_visual, "sample cloud", v1);
	MView->addPointCloud<pcl::PointXYZRGB>(cloud_boundary, "cloud_boundary", v2);

	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_boundary", v2);

	//MView->addCoordinateSystem(1.0);
	MView->initCameraParameters();

	MView->spin();
#endif
	/*pcl::io::savePCDFileBinaryCompressed("all.pcd", *cloud_visual);
	pcl::io::savePCDFileBinaryCompressed("pcd", *cloud_boundary);*/
	return 0;
}

int estimateBorders(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float re, float reforn)
{

	pcl::PointCloud<pcl::Boundary> boundaries;
	//ʹ�ýǶȱ�׼ȷ��һ����Ƿ�λ�ڱ߽��ϡ��ô��������������ݼ���ÿ������Ƶı��淨��
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
	normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud));
	normEst.setRadiusSearch(reforn);
	normEst.compute(*normals);

	boundEst.setInputCloud(cloud);
	boundEst.setInputNormals(normals);
	boundEst.setRadiusSearch(re);
	boundEst.setAngleThreshold(M_PI / 4);
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	boundEst.compute(boundaries);

	for (int i = 0; i < cloud->points.size(); i++)
	{

		if (boundaries[i].boundary_point > 0)
		{
			cloud_boundary->push_back(cloud->points[i]);
		}
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("���Ʊ߽���ȡ"));

	int v1(0);
	MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
	MView->addText("Raw point clouds", 10, 10, "v1_text", v1);
	int v2(0);
	MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
	MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
	MView->addText("Boudary point clouds", 10, 10, "v2_text", v2);

	MView->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
	MView->addPointCloud<pcl::PointXYZ>(cloud_boundary, "cloud_boundary", v2);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_boundary", v2);
	MView->addCoordinateSystem(1.0);
	MView->initCameraParameters();

	MView->spin();

	return 0;
}

int
main(int argc, char** argv)
{
	//���Ե�ǰʱ��Ϊ���ӣ�������������
	//����,time(NULL)������ȡ��ǰʱ�䣬�����ϵõ�����һ����������Ȼ������������������
	//https://www.cnblogs.com/hangaozu/p/8280397.html �����������
	//srand(time(NULL));

	//float re, reforn;
	//re = std::atof(argv[2]);//���ַ���ת���ɸ�����
	//reforn = std::atof(argv[3]);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::io::loadPCDFile(argv[1], *cloud_src);
	//estimateBorders(cloud_src, re, reforn);
	omp_set_num_threads(8);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("test.pcd", *cloud) == -1)		// sac_plane_test.pcd | 800w.pcd | table_scene_lms400.pcd
	{
		PCL_ERROR("���ƶ�ȡʧ�� \n");
		return (-1);
	}

	// ͳ��ѧ�˲�
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;

	filter.setInputCloud(cloud);
	filter.setMeanK(10);
	filter.setStddevMulThresh(0.5);
	filter.filter(*cloudFiltered);
	
	// �����²���
	//pcl::UniformSampling<pcl::PointXYZ> us;
	//us.setInputCloud(cloudFiltered);
	//us.setRadiusSearch(0.01f);
	//us.filter(*cloudFiltered);

	pcl::VoxelGrid<pcl::PointXYZ> vg;

	vg.setInputCloud(cloudFiltered);
	vg.setLeafSize(0.01, 0.01, 0.01);
	vg.filter(*cloudFiltered);

#if ENABLE_TEST
	// �㷨���Կ��
	std::vector<double> vTimes;
	double dMaxTime = 0.0;

	for (int i = 0; i < TESTS_NUM; ++i)
	{
		auto start = std::chrono::high_resolution_clock::now();
		// Angle Criterion�㷨
		//CloudBoundaryAC(cloudFiltered);
		
		// �Ż����Angle Criterion�㷨
		CloudBoundaryACEX(cloudFiltered);

		// Alpha Shape�㷨
		//CloudBoundaryHull(cloudFiltered);

		auto end = std::chrono::high_resolution_clock::now();

		std::chrono::duration<double, std::milli> duration = end - start;
		vTimes.emplace_back(duration.count());

		if (duration.count() > dMaxTime)
		{
			dMaxTime = duration.count();
		}
	}

	double dAverageTime = std::accumulate(vTimes.begin(), vTimes.end(), 0.0) / vTimes.size();
	//std::cout << "AC�߽���ȡƽ����ʱ�� " << dAverageTime << "ms" << std::endl;
	//std::cout << "AC�߽���ȡ�����ʱ�� " << dMaxTime << " ms" << std::endl;

	std::cout << "�Ż����AC�߽���ȡƽ����ʱ�� " << dAverageTime << "ms" << std::endl;
	std::cout << "�Ż����AC�߽���ȡ�����ʱ�� " << dMaxTime << " ms" << std::endl;

	//std::cout << "AS�߽���ȡƽ����ʱ�� " << dAverageTime << "ms" << std::endl;
	//std::cout << "AS�߽���ȡ�����ʱ�� " << dMaxTime << " ms" << std::endl;
#endif

	// ��������
	// Angle Criterion�㷨
	// CloudBoundaryAC(cloudFiltered);
	
	CloudBoundaryACEX(cloudFiltered);

	// Alpha Shape�㷨
	// CloudBoundaryHull(cloudFiltered);

	return 0;
}