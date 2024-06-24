#include <iostream>
#include <algorithm>
#include <vector>

#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/kdtree/kdtree_flann.h>	// kdtree近邻搜索
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h> 
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree.h>

using namespace std;

#define ENABLE_DISPLAY 0				// 定义一个宏，用于控制显示状态
#define ENABLE_TEST 0					// 定义一个宏，用于控制算法测试
#define ENABLE_PRINT 1					// 定义一个宏，用于控制打印信息

const int TESTS_NUM = 20;
const float RADIUS = 0.0001;			// 若两点之间的距离为0.000001则认为是重合点

void RemoveDuplicate1(pcl::PointIndices::Ptr& outliners, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::console::TicToc time;
	time.tic();

	//---------------------KD树半径搜索-------------------
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	pcl::Indices pointIdxR;						// 保存每个近邻点的索引
	vector<float> pointRadiusSquaredDistance;   // 保存每个近邻点与查找点之间的欧式距离平方
	vector<int> total_index;
	/*若某一点在0.000001领域内不止其本身一个点，则认为其有重复点。
	将重复点的索引记录下来，由于后续以此重复点为查询点搜索时，此时这一点也会被定义为重复点，
	但pointIdxRadiusSearch中都是升序排列的，故从pointIdxRadiusSearch中的第二个点的索引开始记录，
	这样可以保证仅仅删除重复的点，保留第一个点*/
	for (size_t i = 0; i < cloud->size(); ++i)//对cloud中的每个点与邻域内的点进行比较
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
	//-----------------------删除重复索引-----------------------
	sort(total_index.begin(), total_index.end());//将索引进行排序
	total_index.erase(unique(total_index.begin(), total_index.end()), total_index.end());//将索引中的重复索引去除

	//-------------------根据索引删除重复的点-------------------
	outliners->indices.resize(total_index.size());
	for (size_t i = 0; i < total_index.size(); i++)
	{
		outliners->indices[i] = total_index[i];
	}

	//-------------------提取删除重复点之后的点云--------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(outliners);
	extract.setNegative(true);//设置为true则表示保存索引之外的点
	extract.filter(*cloud_filtered);

#if ENABLE_PRINT
	cout << "重复点云删除完毕！！！" << endl;
	cout << "去重叠点方法1用时： " << time.toc() << "ms" << endl;
	cout << "原始点云中点的个数为：" << cloud->points.size() << endl;
	cout << "删除的重复点的个数为:" << total_index.size() << endl;
	cout << "去重之后点的个数为:" << cloud_filtered->points.size() << endl;
	cout << "-------------------------------------------------------" << endl;
#endif

#if ENABLE_DISPLAY
	//-------------------------可视化-------------------------
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

	//---------------------------KD树半径搜索---------------------------------
	pcl::KdTreeFLANN<pcl::PointXYZ>  tree;
	tree.setInputCloud(cloud);
	pcl::Indices pointIdxR;  // 保存每个近邻点的索引
	vector<float> Distance;  // 保存每个近邻点与查找点之间的欧式距离平方
	set<int> remove_index;
	//对cloud中的每个点与邻域内的点进行比较
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
	//--------------------------获取重复点的索引-----------------------------
	copy(remove_index.cbegin(), remove_index.cend(), back_inserter(outliners->indices));

	//-------------------提取删除重复点之后的点云--------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(outliners);
	extract.setNegative(true);//设置为true则表示保存索引之外的点
	extract.filter(*cloud_filtered);

#if ENABLE_PRINT
	cout << "重复点云删除完毕！！！" << endl;
	cout << "去重叠点方法2用时： " << time.toc() << "ms" << endl;
	cout << "原始点云中点的个数为：" << cloud->points.size() << endl;
	cout << "删除的重复点的个数为:" << remove_index.size() << endl;
	cout << "去重之后点的个数为:" << cloud_filtered->points.size() << endl;
	cout << "-------------------------------------------------------" << endl;
#endif
#if ENABLE_DISPLAY
	//-------------------------可视化-------------------------
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

	//---------------------------KD树半径搜索---------------------------------
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	pcl::Indices pointIdxR;		// 保存每个近邻点的索引
	vector<float> nn_dists;		// 保存每个近邻点与查找点之间的欧式距离平方
	std::vector<char> label(cloud->size(), '0'); // 初始化点的标签
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// --------------对cloud中的每个点与邻域内的点进行比较-------------------
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
	cout << "去重叠点方法3用时：" << time.toc() << "ms" << endl;
	cout << "原始点云中点的个数为：" << cloud->size() << "个点" << endl;
	cout << "删除的重复点的个数为:" << cloud->size() - filtered->size() << "个" << endl;
	cout << "去重之后点的个数为:" << filtered->size() << "个点" << endl;
	cout << "-------------------------------------------------------" << endl;
#endif

#if ENABLE_DISPLAY
	//---------------------------结果可视化----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName("删除点云中重叠的点");
	viewer->addPointCloud<pcl::PointXYZ>(filtered, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, "sample cloud");// 设置透明度

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

	//---------------------------KD树半径搜索---------------------------------
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	vector<int> pointIdxR;		// 保存每个近邻点的索引
	vector<float> nn_dists;		// 保存每个近邻点与查找点之间的欧式距离平方
	std::vector<bool> label(cloud->size(), false); // 初始化点的标签
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// --------------对cloud中的每个点与邻域内的点进行比较-------------------

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
	cout << "去重叠点方法3X用时：" << time.toc() << "ms" << endl;
	cout << "原始点云中点的个数为：" << cloud->size() << "个点" << endl;
	cout << "删除的重复点的个数为:" << cloud->size() - filtered->size() << "个" << endl;
	cout << "去重之后点的个数为:" << filtered->size() << "个点" << endl;
	cout << "-------------------------------------------------------" << endl;
#endif

#if ENABLE_DISPLAY
	//---------------------------结果可视化----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName("删除点云中重叠的点");
	viewer->addPointCloud<pcl::PointXYZ>(filtered, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, "sample cloud");// 设置透明度

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

	// 创建八叉树对象
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.01); // 设置八叉树分辨率

	// 将点云数据添加到八叉树中
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	pcl::Indices pointIdxR;		// 保存每个近邻点的索引
	vector<float> nn_dists;		// 保存每个近邻点与查找点之间的欧式距离平方
	std::vector<bool> label(cloud->size(), false); // 初始化点的标签
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// --------------对cloud中的每个点与邻域内的点进行比较-------------------
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
	//octree 体素去重叠点
	//std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> voxelCenters;
	//octree.getOccupiedVoxelCenters(voxelCenters);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
	//for (const auto& voxelCenter : voxelCenters)
	//{
	//	filtered->points.emplace_back(voxelCenter.x, voxelCenter.y, voxelCenter.z);
	//}
#if ENABLE_PRINT
	cout << "去重叠点方法4用时：" << time.toc() << "ms" << endl;
	cout << "原始点云中点的个数为：" << cloud->size() << "个点" << endl;
	cout << "删除的重复点的个数为:" << cloud->size() - filtered->size() << "个" << endl;
	cout << "去重之后点的个数为:" << filtered->size() << "个点" << endl;
	cout << "-------------------------------------------------------" << endl;
#endif

#if ENABLE_DISPLAY
	//---------------------------结果可视化----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName("删除点云中重叠的点");
	viewer->addPointCloud<pcl::PointXYZ>(filtered, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.7, "sample cloud");// 设置透明度

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
	//----------------------读取点云---------------------
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
	// 算法测试框架
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
	//std::cout << "PCL去除重叠点平均用时： " << dAverageTime << "ms" << std::endl;
	//std::cout << "PCL去除重叠点最大用时： " << dMaxTime << " ms" << std::endl;

	std::cout << "CAM3C去除重叠点平均用时： " << dAverageTime << "ms" << std::endl;
	std::cout << "CAM3C去除重叠点最大用时： " << dMaxTime << " ms" << std::endl;

#endif

	return 0;
}
