#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/io/pcd_io.h>  
#include <queue>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>
#include <unordered_set>

typedef pcl::PointXYZ PointT;
#define ENABLE_DISPLAY 1		// 定义一个宏，用于控制显示状态

constexpr int RAW_CLOUD_COLOR_R = 63;		// 蓝灰色
constexpr int RAW_CLOUD_COLOR_G = 94;
constexpr int RAW_CLOUD_COLOR_B = 113;

constexpr int REGION_CLOUD_COLOR_R = 136;	// 深红色
constexpr int REGION_CLOUD_COLOR_G = 52;
constexpr int REGION_CLOUD_COLOR_B = 51;

void VisualizeRegion(pcl::PointCloud<PointT>::Ptr cloud, const std::vector<int>& regionIndices) {
	// 创建一个新的点云对象，用于存储当前区域的点
	pcl::PointCloud<PointT>::Ptr regionCloud(new pcl::PointCloud<PointT>());

	// 提取区域中的点
	for (int idx : regionIndices) {
		regionCloud->points.push_back(cloud->points[idx]);
	}
	regionCloud->width = regionCloud->points.size();
	regionCloud->height = 1; // 单行点云
	regionCloud->is_dense = true;

	// 创建可视化器
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Region Viewer"));
	viewer->setBackgroundColor(0, 0, 0); // 黑色背景

	// 显示原始点云
	pcl::visualization::PointCloudColorHandlerCustom<PointT> rawColor(cloud, RAW_CLOUD_COLOR_R, RAW_CLOUD_COLOR_G, RAW_CLOUD_COLOR_B);
	viewer->addPointCloud<PointT>(cloud, rawColor, "Raw cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Raw cloud");

	// 显示区域点云
	pcl::visualization::PointCloudColorHandlerCustom<PointT> regionColor(regionCloud, REGION_CLOUD_COLOR_R, REGION_CLOUD_COLOR_G, REGION_CLOUD_COLOR_B);
	viewer->addPointCloud<PointT>(regionCloud, regionColor, "Region cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Region cloud");

	// 初始化相机参数并重置相机视角
	viewer->resetCamera();

	// 循环显示
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
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

}

void RegionGrowing(pcl::PointCloud<PointT>::Ptr cloud, float radius)
{
	// 构建 k-d Tree
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	std::vector<bool> used(cloud->points.size(), false);  // 标记点是否已被访问
	std::vector<std::vector<int>> regions;               // 存储每个连接项

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
	for (size_t i = 0; i < regions.size(); ++i) {
		std::cout << "Region " << i + 1 << " has " << regions[i].size() << " points." << std::endl;
	}

 #if ENABLE_DISPLAY
	 // 可视化每个连接项
	pcl::visualization::PCLVisualizer viewer("Region Growing Visualization");

	// 为每个区域设置不同颜色
	cout << regions.size() << endl;
	for (size_t i = 0; i < regions.size(); ++i) {
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
#endif
}

void RegionGrowing(std::vector<int>& selectedRegion, const pcl::PointCloud<PointT>::Ptr& cloud, PointT selectedPoint, float radius)
{
	// 构建 k-d Tree
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	// 找到 selectedPoint 在点云中的最近点索引
	std::vector<int> nearestIdx(1);
	std::vector<float> nearestDist(1);
	if (kdtree.nearestKSearch(selectedPoint, 1, nearestIdx, nearestDist) <= 0) {
		std::cerr << "Selected point not found in the cloud!" << std::endl;
		return;
	}

	int startIdx = nearestIdx[0]; // 区域生长的起点索引

	std::vector<bool> used(cloud->points.size(), false);  // 标记点是否已被访问
	std::queue<int> pointsToProcess;                     // 待处理点队列

	// 从选中点开始
	pointsToProcess.push(startIdx);
	used[startIdx] = true;

	// 区域生长
	selectedRegion.clear();
	while (!pointsToProcess.empty()) {
		int currentIdx = pointsToProcess.front();
		pointsToProcess.pop();
		selectedRegion.push_back(currentIdx);  // 加入当前连接项

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
	cout << "区域大小" << selectedRegion.size() << endl;
}

void RegionGrowingOptim(std::vector<int>& selectedRegion, const pcl::PointCloud<PointT>::Ptr& cloud, PointT selectedPoint, float radius)
{
	// 构建 k-d Tree
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	// 找到 selectedPoint 在点云中的最近点索引
	std::vector<int> nearestIdx(1);
	std::vector<float> nearestDist(1);
	if (kdtree.nearestKSearch(selectedPoint, 1, nearestIdx, nearestDist) <= 0) {
		std::cerr << "Selected point not found in the cloud!" << std::endl;
		return;
	}

	int startIdx = nearestIdx[0]; // 区域生长的起点索引

	// 使用 unordered_set 提高查找速度
	std::unordered_set<int> used;
	std::queue<int> pointsToProcess;

	// 从选中点开始
	pointsToProcess.push(startIdx);
	used.insert(startIdx);

	// 区域生长
	selectedRegion.clear();
	selectedRegion.push_back(startIdx);

	while (!pointsToProcess.empty()) {
		int currentIdx = pointsToProcess.front();
		pointsToProcess.pop();

		// 搜索当前点的邻域
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		PointT currentPoint = cloud->points[currentIdx];

		// 使用 k-d tree 进行半径搜索
		if (kdtree.radiusSearch(currentPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
				int neighborIdx = pointIdxRadiusSearch[j];
				if (used.find(neighborIdx) == used.end()) {  // 如果该点未被访问
					pointsToProcess.push(neighborIdx);   // 加入待处理队列
					used.insert(neighborIdx);            // 标记为已访问
					selectedRegion.push_back(neighborIdx); // 加入当前连接项
				}
			}
		}
	}
}

void RegionGrowingParallel(std::vector<int>& selectedRegion, const pcl::PointCloud<PointT>::Ptr& cloud, PointT selectedPoint, float radius)
{
	// 构建 k-d Tree
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	std::vector<int> nearestIdx(1);
	std::vector<float> nearestDist(1);
	kdtree.nearestKSearch(selectedPoint, 1, nearestIdx, nearestDist);

	int startIdx = nearestIdx[0]; // 区域生长的起点索引

	std::unordered_set<int> used;
	std::queue<int> pointsToProcess;
	pointsToProcess.push(startIdx);
	used.insert(startIdx);

	selectedRegion.clear();
	selectedRegion.push_back(startIdx);

#pragma omp parallel
	{
		// 区域生长
		std::queue<int> localQueue;
#pragma omp critical
		{
			localQueue.push(startIdx); // Start from the selected point
		}

		while (!localQueue.empty()) {
			int currentIdx = localQueue.front();
			localQueue.pop();

			// 搜索当前点的邻域
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			PointT currentPoint = cloud->points[currentIdx];

			if (kdtree.radiusSearch(currentPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
				for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
					int neighborIdx = pointIdxRadiusSearch[j];
					if (used.find(neighborIdx) == used.end()) {
#pragma omp critical
						{
							localQueue.push(neighborIdx);
							used.insert(neighborIdx);
							selectedRegion.push_back(neighborIdx);
						}
					}
				}
			}
		}
	}
}

void GetSelectedRegion(std::vector<int>& selectedRegion, const pcl::PointCloud<PointT>::Ptr& cloud, const std::vector<std::vector<int>>& regions, PointT selectedPoint)
{
	// 构建 k-d Tree
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	// 查找选定点的邻域
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// 使用 kdtree 进行半径搜索
	if (kdtree.nearestKSearch(selectedPoint, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
		int neighborIdx = pointIdxRadiusSearch[0];

		// 查找邻近点所属的区域
		for (size_t regionIdx = 0; regionIdx < regions.size(); ++regionIdx) {
			const std::vector<int>& region = regions[regionIdx];

			// 检查邻近点是否属于当前区域
			if (std::find(region.begin(), region.end(), neighborIdx) != region.end()) {
				selectedRegion = region; // 如果找到，返回该区域
				cout << "区域大小" << region.size() << endl;
				break; 
			}
		}
		
	}
}

int main()
{
	//----------------------读取点云---------------------
	pcl::PointCloud<PointT>::Ptr pCloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT>("gyb.pcd", *pCloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}

	//----------------------参数设置---------------------
	std::vector<std::vector<int>> regions;
	//RegionGrowing(regions, pCloud, 2);		// 区域生长法得到不同的区域
	RegionGrowing(pCloud, 2);

	//PointT selectedPoint = PointT(-68.786301, -44.614494, 179.142242);
	PointT selectedPoint = PointT(-43.730217, -75.069366, 195.085403);
	std::vector<int> selectedRegion;
	
	//----------------------程序执行--------------------
	auto startOp = std::chrono::high_resolution_clock::now();
	GetSelectedRegion(selectedRegion, pCloud, regions, selectedPoint);

	//RegionGrowing(pCloud, 2.0);      // 非连接项
	//RegionGrowing(selectedRegion, pCloud, selectedPoint, 2.0);
	//RegionGrowingOptim(selectedRegion, pCloud, selectedPoint, 2.0);
	//RegionGrowingParallel(selectedRegion, pCloud, selectedPoint, 2.0);
	auto endOp = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp = endOp - startOp;
	std::cout << "连接项算法用时: " << elapsedOp.count() << " seconds" << std::endl;

	//----------------------显示结果---------------------
#if ENABLE_DISPLAY
	cout << "点云点数" << pCloud->points.size() << std::endl;
	VisualizeRegion(pCloud, selectedRegion);
#endif
}