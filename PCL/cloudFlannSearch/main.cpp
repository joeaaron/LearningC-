#include "PointSetManager.h"

int main() 
{
	// ����һ��PointSetManager�����趨������ֵ
	double threshold = 0.1; // ������ֵ
	PointSetManager manager(threshold);

	// ʾ������ӵ�һ���㼯
	PointCloudT::Ptr point_set1(new PointCloudT);
	point_set1->points.push_back(PointT(0.0, 0.0, 0.0));
	point_set1->points.push_back(PointT(1.0, 0.0, 0.0));
	manager.AddPointSet(point_set1);

	// ʾ������ӵڶ����㼯
	PointCloudT::Ptr point_set2(new PointCloudT);
	point_set2->points.push_back(PointT(0.05, 0.05, 0.0)); // ��������е㼯�еĵ����ܽ���Ӧ�ò��ᱻ���
	point_set2->points.push_back(PointT(2.0, 0.0, 0.0));   // ��������е㼯�еĵ�����Զ��Ӧ�ûᱻ���
	manager.AddPointSet(point_set2);

	// ��ȡ���е㼯����ӡ
	PointCloudT::Ptr existing_points = manager.GetExistingPoints();
	for (const auto& point : existing_points->points) {
		std::cout << "Point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
	}

	return 0;
}