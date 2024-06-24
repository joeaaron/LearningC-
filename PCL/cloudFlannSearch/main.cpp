#include "PointSetManager.h"

int main() 
{
	// 创建一个PointSetManager对象，设定距离阈值
	double threshold = 0.1; // 距离阈值
	PointSetManager manager(threshold);

	// 示例：添加第一个点集
	PointCloudT::Ptr point_set1(new PointCloudT);
	point_set1->points.push_back(PointT(0.0, 0.0, 0.0));
	point_set1->points.push_back(PointT(1.0, 0.0, 0.0));
	manager.AddPointSet(point_set1);

	// 示例：添加第二个点集
	PointCloudT::Ptr point_set2(new PointCloudT);
	point_set2->points.push_back(PointT(0.05, 0.05, 0.0)); // 这点与现有点集中的点距离很近，应该不会被添加
	point_set2->points.push_back(PointT(2.0, 0.0, 0.0));   // 这点与现有点集中的点距离较远，应该会被添加
	manager.AddPointSet(point_set2);

	// 获取现有点集并打印
	PointCloudT::Ptr existing_points = manager.GetExistingPoints();
	for (const auto& point : existing_points->points) {
		std::cout << "Point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
	}

	return 0;
}