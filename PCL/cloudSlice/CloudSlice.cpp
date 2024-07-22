#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <omp.h> // ����OpenMP֧��

int main(int argc, char** argv)
{
	// ����������������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// ���� cloud �Ѿ������������������Ķ�������

	// �����ȡƽ��Ĳ���
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4); // ƽ�淽�� ax + by + cz + d = 0
	coefficients->values[0] = 1;/* ƽ�淨����x���� */;
	coefficients->values[1] = 0;/* ƽ�淨����y���� */;
	coefficients->values[2] = 0;/* ƽ�淨����z���� */;
	coefficients->values[3] = 1;/* ƽ�浽ԭ��ľ��� */;

	// �����ָ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return -1;
	}

	// ʹ��OpenMP������ȡ�ָ��ĵ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
#pragma omp parallel for
	for (int i = 0; i < inliers->indices.size(); ++i)
	{
		int index = inliers->indices[i];
		pcl::PointXYZ point = cloud->points[index];
#pragma omp critical
		cloud_filtered->push_back(point);
	}

	// cloud_filtered ���ڰ����˽�ȡƽ���ϵĵ�������
	// ������Ҫ��һ������ cloud_filtered

	return 0;
}