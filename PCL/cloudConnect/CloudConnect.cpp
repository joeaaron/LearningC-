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
#define ENABLE_DISPLAY 1		// ����һ���꣬���ڿ�����ʾ״̬

constexpr int RAW_CLOUD_COLOR_R = 63;		// ����ɫ
constexpr int RAW_CLOUD_COLOR_G = 94;
constexpr int RAW_CLOUD_COLOR_B = 113;

constexpr int REGION_CLOUD_COLOR_R = 136;	// ���ɫ
constexpr int REGION_CLOUD_COLOR_G = 52;
constexpr int REGION_CLOUD_COLOR_B = 51;

void VisualizeRegion(pcl::PointCloud<PointT>::Ptr cloud, const std::vector<int>& regionIndices) {
	// ����һ���µĵ��ƶ������ڴ洢��ǰ����ĵ�
	pcl::PointCloud<PointT>::Ptr regionCloud(new pcl::PointCloud<PointT>());

	// ��ȡ�����еĵ�
	for (int idx : regionIndices) {
		regionCloud->points.push_back(cloud->points[idx]);
	}
	regionCloud->width = regionCloud->points.size();
	regionCloud->height = 1; // ���е���
	regionCloud->is_dense = true;

	// �������ӻ���
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Region Viewer"));
	viewer->setBackgroundColor(0, 0, 0); // ��ɫ����

	// ��ʾԭʼ����
	pcl::visualization::PointCloudColorHandlerCustom<PointT> rawColor(cloud, RAW_CLOUD_COLOR_R, RAW_CLOUD_COLOR_G, RAW_CLOUD_COLOR_B);
	viewer->addPointCloud<PointT>(cloud, rawColor, "Raw cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Raw cloud");

	// ��ʾ�������
	pcl::visualization::PointCloudColorHandlerCustom<PointT> regionColor(regionCloud, REGION_CLOUD_COLOR_R, REGION_CLOUD_COLOR_G, REGION_CLOUD_COLOR_B);
	viewer->addPointCloud<PointT>(regionCloud, regionColor, "Region cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Region cloud");

	// ��ʼ�������������������ӽ�
	viewer->resetCamera();

	// ѭ����ʾ
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
	}
}

void RegionGrowing(std::vector<std::vector<int>>& regions, const pcl::PointCloud<PointT>::Ptr& cloud, float radius)
{
	// ���� k-d Tree
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	std::vector<bool> used(cloud->points.size(), false);  // ��ǵ��Ƿ��ѱ�����
	regions.swap(std::vector<std::vector<int>>());
	// �������е㣬Ѱ��������
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		if (used[i]) continue;  // ������Ѿ������ʹ�������

		std::vector<int> currentRegion;    // ��ǰ������
		std::queue<int> pointsToProcess;   // ����������
		pointsToProcess.push(i);          // �ӵ�ǰ�㿪ʼ����
		used[i] = true;                   // ���Ϊ�ѷ���

		// ��������
		while (!pointsToProcess.empty()) {
			int currentIdx = pointsToProcess.front();
			pointsToProcess.pop();

			currentRegion.push_back(currentIdx);  // ���뵱ǰ������

			// ������ǰ�������
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			PointT currentPoint = cloud->points[currentIdx];

			if (kdtree.radiusSearch(currentPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
				for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
					int neighborIdx = pointIdxRadiusSearch[j];
					if (!used[neighborIdx]) {
						pointsToProcess.push(neighborIdx);  // ������������
						used[neighborIdx] = true;          // ���Ϊ�ѷ���
					}
				}
			}
		}

		// ���ҵ���������洢����
		if (!currentRegion.empty()) {
			regions.push_back(currentRegion);
		}
	}

}

void RegionGrowing(pcl::PointCloud<PointT>::Ptr cloud, float radius)
{
	// ���� k-d Tree
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	std::vector<bool> used(cloud->points.size(), false);  // ��ǵ��Ƿ��ѱ�����
	std::vector<std::vector<int>> regions;               // �洢ÿ��������

	// �������е㣬Ѱ��������
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		if (used[i]) continue;  // ������Ѿ������ʹ�������

		std::vector<int> currentRegion;    // ��ǰ������
		std::queue<int> pointsToProcess;   // ����������
		pointsToProcess.push(i);          // �ӵ�ǰ�㿪ʼ����
		used[i] = true;                   // ���Ϊ�ѷ���

		// ��������
		while (!pointsToProcess.empty()) {
			int currentIdx = pointsToProcess.front();
			pointsToProcess.pop();

			currentRegion.push_back(currentIdx);  // ���뵱ǰ������

			// ������ǰ�������
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			PointT currentPoint = cloud->points[currentIdx];

			if (kdtree.radiusSearch(currentPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
				for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
					int neighborIdx = pointIdxRadiusSearch[j];
					if (!used[neighborIdx]) {
						pointsToProcess.push(neighborIdx);  // ������������
						used[neighborIdx] = true;          // ���Ϊ�ѷ���
					}
				}
			}
		}

		// ���ҵ���������洢����
		if (!currentRegion.empty()) {
			regions.push_back(currentRegion);
		}
	}

	// �� regions ����ÿ������������ĵ����Ӵ�С����
	std::sort(regions.begin(), regions.end(),
		[](const std::vector<int>& a, const std::vector<int>& b) {
			return a.size() > b.size();  // �Ӵ�С����
		});

	// �����������������Ϣ
	for (size_t i = 0; i < regions.size(); ++i) {
		std::cout << "Region " << i + 1 << " has " << regions[i].size() << " points." << std::endl;
	}

 #if ENABLE_DISPLAY
	 // ���ӻ�ÿ��������
	pcl::visualization::PCLVisualizer viewer("Region Growing Visualization");

	// Ϊÿ���������ò�ͬ��ɫ
	cout << regions.size() << endl;
	for (size_t i = 0; i < regions.size(); ++i) {
		// �����µĵ��ƶ������洢��ǰ����
		pcl::PointCloud<PointT>::Ptr regionCloud(new pcl::PointCloud<PointT>());

		// ����ǰ��������ĵ�����µĵ�����
		for (size_t j = 0; j < regions[i].size(); ++j) {
			regionCloud->points.push_back(cloud->points[regions[i][j]]);
		}

		// Ϊ������ָ��һ�������ɫ
		float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		float b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

		// ���ӻ���ǰ����
		std::string regionId = "region_" + std::to_string(i);
		viewer.addPointCloud(regionCloud, regionId);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, regionId);
	}

	// ������ͼ����
	viewer.setBackgroundColor(0, 0, 0);  // ���ñ���ɫΪ��ɫ
	viewer.addCoordinateSystem(1.0);     // ���������
	viewer.initCameraParameters();       // ��ʼ���������

	// �������ӻ�
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);  // ���ϸ�����ͼ
	}
#endif
}

void RegionGrowing(std::vector<int>& selectedRegion, const pcl::PointCloud<PointT>::Ptr& cloud, PointT selectedPoint, float radius)
{
	// ���� k-d Tree
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	// �ҵ� selectedPoint �ڵ����е����������
	std::vector<int> nearestIdx(1);
	std::vector<float> nearestDist(1);
	if (kdtree.nearestKSearch(selectedPoint, 1, nearestIdx, nearestDist) <= 0) {
		std::cerr << "Selected point not found in the cloud!" << std::endl;
		return;
	}

	int startIdx = nearestIdx[0]; // �����������������

	std::vector<bool> used(cloud->points.size(), false);  // ��ǵ��Ƿ��ѱ�����
	std::queue<int> pointsToProcess;                     // ����������

	// ��ѡ�е㿪ʼ
	pointsToProcess.push(startIdx);
	used[startIdx] = true;

	// ��������
	selectedRegion.clear();
	while (!pointsToProcess.empty()) {
		int currentIdx = pointsToProcess.front();
		pointsToProcess.pop();
		selectedRegion.push_back(currentIdx);  // ���뵱ǰ������

		// ������ǰ�������
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		PointT currentPoint = cloud->points[currentIdx];

		if (kdtree.radiusSearch(currentPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
				int neighborIdx = pointIdxRadiusSearch[j];
				if (!used[neighborIdx]) {
					pointsToProcess.push(neighborIdx);  // ������������
					used[neighborIdx] = true;          // ���Ϊ�ѷ���
				}
			}
		}
	}
	cout << "�����С" << selectedRegion.size() << endl;
}

void RegionGrowingOptim(std::vector<int>& selectedRegion, const pcl::PointCloud<PointT>::Ptr& cloud, PointT selectedPoint, float radius)
{
	// ���� k-d Tree
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	// �ҵ� selectedPoint �ڵ����е����������
	std::vector<int> nearestIdx(1);
	std::vector<float> nearestDist(1);
	if (kdtree.nearestKSearch(selectedPoint, 1, nearestIdx, nearestDist) <= 0) {
		std::cerr << "Selected point not found in the cloud!" << std::endl;
		return;
	}

	int startIdx = nearestIdx[0]; // �����������������

	// ʹ�� unordered_set ��߲����ٶ�
	std::unordered_set<int> used;
	std::queue<int> pointsToProcess;

	// ��ѡ�е㿪ʼ
	pointsToProcess.push(startIdx);
	used.insert(startIdx);

	// ��������
	selectedRegion.clear();
	selectedRegion.push_back(startIdx);

	while (!pointsToProcess.empty()) {
		int currentIdx = pointsToProcess.front();
		pointsToProcess.pop();

		// ������ǰ�������
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		PointT currentPoint = cloud->points[currentIdx];

		// ʹ�� k-d tree ���а뾶����
		if (kdtree.radiusSearch(currentPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
				int neighborIdx = pointIdxRadiusSearch[j];
				if (used.find(neighborIdx) == used.end()) {  // ����õ�δ������
					pointsToProcess.push(neighborIdx);   // ������������
					used.insert(neighborIdx);            // ���Ϊ�ѷ���
					selectedRegion.push_back(neighborIdx); // ���뵱ǰ������
				}
			}
		}
	}
}

void RegionGrowingParallel(std::vector<int>& selectedRegion, const pcl::PointCloud<PointT>::Ptr& cloud, PointT selectedPoint, float radius)
{
	// ���� k-d Tree
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	std::vector<int> nearestIdx(1);
	std::vector<float> nearestDist(1);
	kdtree.nearestKSearch(selectedPoint, 1, nearestIdx, nearestDist);

	int startIdx = nearestIdx[0]; // �����������������

	std::unordered_set<int> used;
	std::queue<int> pointsToProcess;
	pointsToProcess.push(startIdx);
	used.insert(startIdx);

	selectedRegion.clear();
	selectedRegion.push_back(startIdx);

#pragma omp parallel
	{
		// ��������
		std::queue<int> localQueue;
#pragma omp critical
		{
			localQueue.push(startIdx); // Start from the selected point
		}

		while (!localQueue.empty()) {
			int currentIdx = localQueue.front();
			localQueue.pop();

			// ������ǰ�������
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
	// ���� k-d Tree
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);

	// ����ѡ���������
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// ʹ�� kdtree ���а뾶����
	if (kdtree.nearestKSearch(selectedPoint, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
		int neighborIdx = pointIdxRadiusSearch[0];

		// �����ڽ�������������
		for (size_t regionIdx = 0; regionIdx < regions.size(); ++regionIdx) {
			const std::vector<int>& region = regions[regionIdx];

			// ����ڽ����Ƿ����ڵ�ǰ����
			if (std::find(region.begin(), region.end(), neighborIdx) != region.end()) {
				selectedRegion = region; // ����ҵ������ظ�����
				cout << "�����С" << region.size() << endl;
				break; 
			}
		}
		
	}
}

int main()
{
	//----------------------��ȡ����---------------------
	pcl::PointCloud<PointT>::Ptr pCloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT>("gyb.pcd", *pCloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}

	//----------------------��������---------------------
	std::vector<std::vector<int>> regions;
	//RegionGrowing(regions, pCloud, 2);		// �����������õ���ͬ������
	RegionGrowing(pCloud, 2);

	//PointT selectedPoint = PointT(-68.786301, -44.614494, 179.142242);
	PointT selectedPoint = PointT(-43.730217, -75.069366, 195.085403);
	std::vector<int> selectedRegion;
	
	//----------------------����ִ��--------------------
	auto startOp = std::chrono::high_resolution_clock::now();
	GetSelectedRegion(selectedRegion, pCloud, regions, selectedPoint);

	//RegionGrowing(pCloud, 2.0);      // ��������
	//RegionGrowing(selectedRegion, pCloud, selectedPoint, 2.0);
	//RegionGrowingOptim(selectedRegion, pCloud, selectedPoint, 2.0);
	//RegionGrowingParallel(selectedRegion, pCloud, selectedPoint, 2.0);
	auto endOp = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp = endOp - startOp;
	std::cout << "�������㷨��ʱ: " << elapsedOp.count() << " seconds" << std::endl;

	//----------------------��ʾ���---------------------
#if ENABLE_DISPLAY
	cout << "���Ƶ���" << pCloud->points.size() << std::endl;
	VisualizeRegion(pCloud, selectedRegion);
#endif
}