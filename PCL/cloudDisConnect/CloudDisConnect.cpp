#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>  
#include <queue>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

typedef pcl::PointXYZ PointT;
const int SENSITIVITY = 13;
#define ENABLE_DISPLAY 1		// 定义一个宏，用于控制显示状态

void VisualizeRegion(const pcl::PointCloud<PointT>::Ptr& cloud, const std::vector<std::vector<int>>& regions) {
	// 可视化每个连接项
	pcl::visualization::PCLVisualizer viewer("Region Growing Visualization");
	viewer.addPointCloud<PointT>(cloud, "Raw cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.2627, 0.3451, 0.4588, "Raw cloud");

	// 为每个区域设置不同颜色
	/*cout << regions.size() << endl;*/
	for (size_t i = 2; i < regions.size(); ++i) {
		// 创建新的点云对象来存储当前区域
		pcl::PointCloud<PointT>::Ptr regionCloud(new pcl::PointCloud<PointT>());

		// 将当前连接区域的点加入新的点云中
		for (size_t j = 0; j < regions[i].size(); ++j) {
			regionCloud->points.push_back(cloud->points[regions[i][j]]);
		}

		// 为该区域指定一个随机颜色
		float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

		// 可视化当前区域
		std::string regionId = "region_" + std::to_string(i);
		viewer.addPointCloud(regionCloud, regionId);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, regionId);
	}

	// 设置视图属性
	viewer.setBackgroundColor(0, 0, 0);  // 设置背景色为黑色
	viewer.addCoordinateSystem(1.0);     // 添加坐标轴
	viewer.initCameraParameters();       // 初始化相机参数

	// 启动可视化
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);  // 不断更新视图
	}
}

void RegionGrowing(std::vector<std::vector<int>>& regions, const pcl::PointCloud<PointT>::Ptr& cloud, float radius)
{
	// 构建 k-d Tree
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	std::vector<bool> used(cloud->points.size(), false);  // 标记点是否已被访问
	regions.swap(std::vector<std::vector<int>>());
	// 遍历所有点，寻找连接项
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		if (used[i]) continue;  // 如果点已经被访问过，跳过

		std::vector<int> currentRegion;    // 当前连接项
		std::queue<int> pointsToProcess;   // 待处理点队列
		pointsToProcess.push(i);          // 从当前点开始生长
		used[i] = true;                   // 标记为已访问

		// 区域生长
		while (!pointsToProcess.empty()) {
			int currentIdx = pointsToProcess.front();
			pointsToProcess.pop();

			currentRegion.push_back(currentIdx);  // 加入当前连接项

			// 搜索当前点的邻域
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			PointT currentPoint = cloud->points[currentIdx];

			if (kdtree.radiusSearch(currentPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
				for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
					int neighborIdx = pointIdxRadiusSearch[j];
					if (!used[neighborIdx]) {
						pointsToProcess.push(neighborIdx);  // 加入待处理队列
						used[neighborIdx] = true;          // 标记为已访问
					}
				}
			}
		}

		// 将找到的连接项存储起来
		if (!currentRegion.empty()) {
			regions.push_back(currentRegion);
		}
	}

	// 对 regions 按照每个连接项包含的点数从大到小排序
	std::sort(regions.begin(), regions.end(),
		[](const std::vector<int>& a, const std::vector<int>& b) {
			return a.size() > b.size();  // 从大到小排序
		});

	// 输出排序后的连接项信息
	int allRegionSize = 0;
	for (size_t i = 0; i < regions.size(); ++i) {
		std::cout << "Region " << i + 1 << " has " << regions[i].size() << " points." << std::endl;
		allRegionSize += regions[i].size();
	}
	std::cout << "Size" << allRegionSize << endl;
}

int main()
{
	//----------------------读取点云---------------------
	pcl::PointCloud<PointT>::Ptr pCloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT>("2.pcd", *pCloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}

	//----------------------参数设置---------------------
	std::vector<std::vector<int>> disConnectedRegions;
	RegionGrowing(disConnectedRegions, pCloud, 2.0);

	//----------------------程序执行--------------------
	auto startOp = std::chrono::high_resolution_clock::now();
	//RegionGrowing(disConnectedRegions, pCloud, 2.0);
	auto endOp = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp = endOp - startOp;
	std::cout << "连接项算法用时: " << elapsedOp.count() << " seconds" << std::endl;

	//----------------------显示结果---------------------
#if ENABLE_DISPLAY
	cout << "点云点数" << pCloud->points.size() << std::endl;
	VisualizeRegion(pCloud, disConnectedRegions);
#endif
}