#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>  
#include <queue>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

typedef pcl::PointXYZ PointT;
const int SENSITIVITY = 13;
#define ENABLE_DISPLAY 1		// ����һ���꣬���ڿ�����ʾ״̬

void VisualizeRegion(const pcl::PointCloud<PointT>::Ptr& cloud, const std::vector<std::vector<int>>& regions) {
	// ���ӻ�ÿ��������
	pcl::visualization::PCLVisualizer viewer("Region Growing Visualization");
	viewer.addPointCloud<PointT>(cloud, "Raw cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.2627, 0.3451, 0.4588, "Raw cloud");

	// Ϊÿ���������ò�ͬ��ɫ
	/*cout << regions.size() << endl;*/
	for (size_t i = 2; i < regions.size(); ++i) {
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

	// �� regions ����ÿ������������ĵ����Ӵ�С����
	std::sort(regions.begin(), regions.end(),
		[](const std::vector<int>& a, const std::vector<int>& b) {
			return a.size() > b.size();  // �Ӵ�С����
		});

	// �����������������Ϣ
	int allRegionSize = 0;
	for (size_t i = 0; i < regions.size(); ++i) {
		std::cout << "Region " << i + 1 << " has " << regions[i].size() << " points." << std::endl;
		allRegionSize += regions[i].size();
	}
	std::cout << "Size" << allRegionSize << endl;
}

int main()
{
	//----------------------��ȡ����---------------------
	pcl::PointCloud<PointT>::Ptr pCloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT>("2.pcd", *pCloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}

	//----------------------��������---------------------
	std::vector<std::vector<int>> disConnectedRegions;
	RegionGrowing(disConnectedRegions, pCloud, 2.0);

	//----------------------����ִ��--------------------
	auto startOp = std::chrono::high_resolution_clock::now();
	//RegionGrowing(disConnectedRegions, pCloud, 2.0);
	auto endOp = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp = endOp - startOp;
	std::cout << "�������㷨��ʱ: " << elapsedOp.count() << " seconds" << std::endl;

	//----------------------��ʾ���---------------------
#if ENABLE_DISPLAY
	cout << "���Ƶ���" << pCloud->points.size() << std::endl;
	VisualizeRegion(pCloud, disConnectedRegions);
#endif
}