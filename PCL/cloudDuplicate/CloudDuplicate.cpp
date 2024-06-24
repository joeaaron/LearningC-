#include <iostream>
#include <algorithm>
#include <vector>

#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/kdtree/kdtree_flann.h>	// kdtree��������
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h> 
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree.h>

using namespace std;

#define ENABLE_DISPLAY 0				// ����һ���꣬���ڿ�����ʾ״̬
#define ENABLE_TEST 0					// ����һ���꣬���ڿ����㷨����
#define ENABLE_PRINT 1					// ����һ���꣬���ڿ��ƴ�ӡ��Ϣ

const int TESTS_NUM = 20;
const float RADIUS = 0.0001;			// ������֮��ľ���Ϊ0.000001����Ϊ���غϵ�

void RemoveDuplicate1(pcl::PointIndices::Ptr& outliners, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::console::TicToc time;
	time.tic();

	//---------------------KD���뾶����-------------------
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	pcl::Indices pointIdxR;						// ����ÿ�����ڵ������
	vector<float> pointRadiusSquaredDistance;   // ����ÿ�����ڵ�����ҵ�֮���ŷʽ����ƽ��
	vector<int> total_index;
	/*��ĳһ����0.000001�����ڲ�ֹ�䱾��һ���㣬����Ϊ�����ظ��㡣
	���ظ����������¼���������ں����Դ��ظ���Ϊ��ѯ������ʱ����ʱ��һ��Ҳ�ᱻ����Ϊ�ظ��㣬
	��pointIdxRadiusSearch�ж����������еģ��ʴ�pointIdxRadiusSearch�еĵڶ������������ʼ��¼��
	�������Ա�֤����ɾ���ظ��ĵ㣬������һ����*/
	for (size_t i = 0; i < cloud->size(); ++i)//��cloud�е�ÿ�����������ڵĵ���бȽ�
	{
		pcl::PointXYZ searchPoint = cloud->points[i];

		if (kdtree.radiusSearch(searchPoint, RADIUS, pointIdxR, pointRadiusSquaredDistance) > 0)
		{
			if (pointIdxR.size() != 1)
			{
				for (size_t j = 1; j < pointIdxR.size(); j++)
				{
					total_index.push_back(pointIdxR[j]);
				}
			}
		}
	}
	//-----------------------ɾ���ظ�����-----------------------
	sort(total_index.begin(), total_index.end());//��������������
	total_index.erase(unique(total_index.begin(), total_index.end()), total_index.end());//�������е��ظ�����ȥ��

	//-------------------��������ɾ���ظ��ĵ�-------------------
	outliners->indices.resize(total_index.size());
	for (size_t i = 0; i < total_index.size(); i++)
	{
		outliners->indices[i] = total_index[i];
	}

	//-------------------��ȡɾ���ظ���֮��ĵ���--------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(outliners);
	extract.setNegative(true);//����Ϊtrue���ʾ��������֮��ĵ�
	extract.filter(*cloud_filtered);

#if ENABLE_PRINT
	cout << "�ظ�����ɾ����ϣ�����" << endl;
	cout << "ȥ�ص��㷽��1��ʱ�� " << time.toc() << "ms" << endl;
	cout << "ԭʼ�����е�ĸ���Ϊ��" << cloud->points.size() << endl;
	cout << "ɾ�����ظ���ĸ���Ϊ:" << total_index.size() << endl;
	cout << "ȥ��֮���ĸ���Ϊ:" << cloud_filtered->points.size() << endl;
	cout << "-------------------------------------------------------" << endl;
#endif

#if ENABLE_DISPLAY
	//-------------------------���ӻ�-------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_filtered, 0, 255, 0); // green

	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, single_color, "sample cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
}

void RemoveDuplicate2(pcl::PointIndices::Ptr& outliners, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::console::TicToc time;
	time.tic();

	//---------------------------KD���뾶����---------------------------------
	pcl::KdTreeFLANN<pcl::PointXYZ>  tree;
	tree.setInputCloud(cloud);
	pcl::Indices pointIdxR;  // ����ÿ�����ڵ������
	vector<float> Distance;  // ����ÿ�����ڵ�����ҵ�֮���ŷʽ����ƽ��
	set<int> remove_index;
	//��cloud�е�ÿ�����������ڵĵ���бȽ�
	for (auto& poiont_i : *cloud)
	{
		if (tree.radiusSearch(poiont_i, RADIUS, pointIdxR, Distance) > 0)
		{
			if (pointIdxR.size() != 1)
			{
				for (size_t i = 1; i < pointIdxR.size(); ++i)
				{
					remove_index.insert(pointIdxR[i]);
				}
			}
		}
	}
	//--------------------------��ȡ�ظ��������-----------------------------
	copy(remove_index.cbegin(), remove_index.cend(), back_inserter(outliners->indices));

	//-------------------��ȡɾ���ظ���֮��ĵ���--------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(outliners);
	extract.setNegative(true);//����Ϊtrue���ʾ��������֮��ĵ�
	extract.filter(*cloud_filtered);

#if ENABLE_PRINT
	cout << "�ظ�����ɾ����ϣ�����" << endl;
	cout << "ȥ�ص��㷽��2��ʱ�� " << time.toc() << "ms" << endl;
	cout << "ԭʼ�����е�ĸ���Ϊ��" << cloud->points.size() << endl;
	cout << "ɾ�����ظ���ĸ���Ϊ:" << remove_index.size() << endl;
	cout << "ȥ��֮���ĸ���Ϊ:" << cloud_filtered->points.size() << endl;
	cout << "-------------------------------------------------------" << endl;
#endif
#if ENABLE_DISPLAY
	//-------------------------���ӻ�-------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_filtered, 0, 255, 0); // green

	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, single_color, "sample cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
}

void RemoveDuplicate3(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::console::TicToc time;
	time.tic();

	//---------------------------KD���뾶����---------------------------------
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	pcl::Indices pointIdxR;		// ����ÿ�����ڵ������
	vector<float> nn_dists;		// ����ÿ�����ڵ�����ҵ�֮���ŷʽ����ƽ��
	std::vector<char> label(cloud->size(), '0'); // ��ʼ����ı�ǩ
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// --------------��cloud�е�ÿ�����������ڵĵ���бȽ�-------------------
	for (int i = 0; i < cloud->size(); ++i)
	{
		if (label[i] == '1')
		{
			continue;
		}
		if (tree.radiusSearch(cloud->points[i], RADIUS, pointIdxR, nn_dists) > 0)
		{
			for (auto& pi : pointIdxR)
			{
				label[pi] = '1';
			}
		}

		filtered->push_back(cloud->points[i]);
	}

#if ENABLE_PRINT
	cout << "ȥ�ص��㷽��3��ʱ��" << time.toc() << "ms" << endl;
	cout << "ԭʼ�����е�ĸ���Ϊ��" << cloud->size() << "����" << endl;
	cout << "ɾ�����ظ���ĸ���Ϊ:" << cloud->size() - filtered->size() << "��" << endl;
	cout << "ȥ��֮���ĸ���Ϊ:" << filtered->size() << "����" << endl;
	cout << "-------------------------------------------------------" << endl;
#endif

#if ENABLE_DISPLAY
	//---------------------------������ӻ�----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName("ɾ���������ص��ĵ�");
	viewer->addPointCloud<pcl::PointXYZ>(filtered, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, "sample cloud");// ����͸����

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
}

void RemoveDuplicate3X(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::console::TicToc time;
	time.tic();

	//---------------------------KD���뾶����---------------------------------
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	vector<int> pointIdxR;		// ����ÿ�����ڵ������
	vector<float> nn_dists;		// ����ÿ�����ڵ�����ҵ�֮���ŷʽ����ƽ��
	std::vector<bool> label(cloud->size(), false); // ��ʼ����ı�ǩ
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// --------------��cloud�е�ÿ�����������ڵĵ���бȽ�-------------------

	for (int i = 0; i < cloud->size(); ++i)
	{
		if (label[i])
		{
			continue;
		}
		if (tree.radiusSearch(cloud->points[i], RADIUS, pointIdxR, nn_dists) > 0)
		{
			for (int j = 0; j < pointIdxR.size(); ++j)
			{
				label[pointIdxR[j]] = true;
			}
		}

		filtered->push_back(cloud->points[i]);
	}

#if ENABLE_PRINT
	cout << "ȥ�ص��㷽��3X��ʱ��" << time.toc() << "ms" << endl;
	cout << "ԭʼ�����е�ĸ���Ϊ��" << cloud->size() << "����" << endl;
	cout << "ɾ�����ظ���ĸ���Ϊ:" << cloud->size() - filtered->size() << "��" << endl;
	cout << "ȥ��֮���ĸ���Ϊ:" << filtered->size() << "����" << endl;
	cout << "-------------------------------------------------------" << endl;
#endif

#if ENABLE_DISPLAY
	//---------------------------������ӻ�----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName("ɾ���������ص��ĵ�");
	viewer->addPointCloud<pcl::PointXYZ>(filtered, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, "sample cloud");// ����͸����

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
}

void RemoveDuplicate4(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::console::TicToc time;
	time.tic();

	// �����˲�������
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.01); // ���ð˲����ֱ���

	// ������������ӵ��˲�����
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	pcl::Indices pointIdxR;		// ����ÿ�����ڵ������
	vector<float> nn_dists;		// ����ÿ�����ڵ�����ҵ�֮���ŷʽ����ƽ��
	std::vector<bool> label(cloud->size(), false); // ��ʼ����ı�ǩ
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// --------------��cloud�е�ÿ�����������ڵĵ���бȽ�-------------------
	for (int i = 0; i < cloud->size(); ++i)
	{
		if (label[i])
		{
			continue;
		}
		if (octree.radiusSearch(cloud->points[i], RADIUS, pointIdxR, nn_dists) > 0)
		{
			for (auto& pi : pointIdxR)
			{
				label[pi] = true;
			}
		}

		filtered->push_back(cloud->points[i]);
	}
	//octree ����ȥ�ص���
	//std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> voxelCenters;
	//octree.getOccupiedVoxelCenters(voxelCenters);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
	//for (const auto& voxelCenter : voxelCenters)
	//{
	//	filtered->points.emplace_back(voxelCenter.x, voxelCenter.y, voxelCenter.z);
	//}
#if ENABLE_PRINT
	cout << "ȥ�ص��㷽��4��ʱ��" << time.toc() << "ms" << endl;
	cout << "ԭʼ�����е�ĸ���Ϊ��" << cloud->size() << "����" << endl;
	cout << "ɾ�����ظ���ĸ���Ϊ:" << cloud->size() - filtered->size() << "��" << endl;
	cout << "ȥ��֮���ĸ���Ϊ:" << filtered->size() << "����" << endl;
	cout << "-------------------------------------------------------" << endl;
#endif

#if ENABLE_DISPLAY
	//---------------------------������ӻ�----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName("ɾ���������ص��ĵ�");
	viewer->addPointCloud<pcl::PointXYZ>(filtered, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, "sample cloud");// ����͸����

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
}

template <typename PointType>
bool RemoveDuplicatedForPointCloud(std::vector<PointType>& vptPointsInOut,
	const std::vector<PointType>& vptDuplicatedPoints,
	const double dDistThresh = 0.0001)
{
	try
	{
		if (vptDuplicatedPoints.empty() || vptPointsInOut.empty())
		{
			return false;
		}

		scKDTree<PointType> kdtree;
		kdtree.SetInputDataAndIndexes(&vptPointsInOut);
		if (!kdtree.Execute())
		{
			return false;
		}

		std::vector<int> vnIndices;
		std::vector<double> vdDists;
		std::vector<bool> vbFlags(vptPointsInOut.size(), true);

		for (int i = 0; i < vptDuplicatedPoints.size(); ++i)
		{
			kdtree.RadiusSearch(vnIndices, vdDists, vptDuplicatedPoints[i], dDistThresh);
			if (vnIndices.size() > 1)
			{
				for (int n = 0; n < vnIndices.size(); ++n)
				{
					vbFlags[vnIndices[n]] = false;
				}
			}
		}

		std::vector<PointType> vptPointsValid;
		vptPointsValid.reserve(vptPointsInOut.size());

		for (int i = 0; i < vbFlags.size(); ++i)
		{
			if (vbFlags[i])
			{
				vptPointsValid.emplace_back(vptPointsInOut[i]);
			}
		}
		vptPointsValid.shrink_to_fit();
		vptPointsValid.swap(vptPointsInOut);
	}
	catch (...)
	{
		return false;
	}

	return true;
}

int main()
{
	//----------------------��ȡ����---------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("1600w.pcd", *cloud) == -1)			// bunnyDuplicate.pcd
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}

	pcl::PointIndices::Ptr outliners(new pcl::PointIndices()); 
	vector<int> total_index;

	//RemoveDuplicate1(outliners, cloud);
	//RemoveDuplicate2(outliners, cloud);
	//RemoveDuplicate3(cloud);
	//RemoveDuplicate3X(cloud);
	RemoveDuplicate4(cloud);

#if ENABLE_TEST
	// �㷨���Կ��
	std::vector<double> vTimes;
	double dMaxTime = 0.0;

	for (int i = 0; i < TESTS_NUM; ++i)
	{
		auto start = std::chrono::high_resolution_clock::now();
		RemoveDuplicate1(outliners, cloud);
		//RemoveDuplicate3X(cloud);
		auto end = std::chrono::high_resolution_clock::now();

		std::chrono::duration<double, std::milli> duration = end - start;
		vTimes.emplace_back(duration.count());

		if (duration.count() > dMaxTime)
		{
			dMaxTime = duration.count();
		}
	}

	double dAverageTime = std::accumulate(vTimes.begin(), vTimes.end(), 0.0) / vTimes.size();
	//std::cout << "PCLȥ���ص���ƽ����ʱ�� " << dAverageTime << "ms" << std::endl;
	//std::cout << "PCLȥ���ص��������ʱ�� " << dMaxTime << " ms" << std::endl;

	std::cout << "CAM3Cȥ���ص���ƽ����ʱ�� " << dAverageTime << "ms" << std::endl;
	std::cout << "CAM3Cȥ���ص��������ʱ�� " << dMaxTime << " ms" << std::endl;

#endif

	return 0;
}
