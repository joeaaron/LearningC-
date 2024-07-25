#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;
const double THRESH = 0.5;

void SplitPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// Perform PCA on the input cloud
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloud);

	Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
	Eigen::Vector3f principal_direction = eigenvectors.col(0); // Main direction (X-axis)
	Eigen::Vector3f secondary_direction = eigenvectors.col(1); // Secondary direction (Y-axis)

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);

	// Prepare indices for splitting
	pcl::PointIndices::Ptr indices_left(new pcl::PointIndices);
	pcl::PointIndices::Ptr indices_right(new pcl::PointIndices);
	pcl::PointIndices::Ptr indices_upper(new pcl::PointIndices);
	pcl::PointIndices::Ptr indices_lower(new pcl::PointIndices);

	Eigen::Vector4f minPoint, maxPoint;
	pcl::getMinMax3D(*cloud, minPoint, maxPoint);
	double minX = minPoint.x();
	double maxX = maxPoint.x();

	double minZ = minPoint.z();
	double maxZ = maxPoint.z();
	// Split the cloud based on the principal and secondary directions
	for (size_t i = 0; i < cloud->points.size(); ++i) 
	{
		const pcl::PointXYZ& point = cloud->points[i];
		Eigen::Vector3f point_vector(point.x - centroid[0], point.y - centroid[1], point.z - centroid[2]);
		float projection_main = point_vector.dot(principal_direction);
		float projection_secondary = point_vector.dot(secondary_direction);

		//if (projection_main > 0) { // Right side
		//	if (projection_secondary > 0) { // Upper
		//		indices_right_upper->indices.push_back(i);
		//	}
		//	else { // Lower
		//		indices_right_lower->indices.push_back(i);
		//	}
		//}
		//else { // Left side
		//	if (projection_secondary > 0) { // Upper
		//		indices_left_upper->indices.push_back(i);
		//	}
		//	else { // Lower
		//		indices_left_lower->indices.push_back(i);
		//	}
		//}
		if (projection_main < 0 && std::abs(point.x - minX) < THRESH)
		{
			indices_left->indices.push_back(i);
		}
		else if (projection_main > 0 && std::abs(point.x - maxX) < THRESH)
		{
			indices_right->indices.push_back(i);
		}

		if (projection_secondary < 0 && std::abs(point.z - minZ) < THRESH)
		{
			indices_lower->indices.push_back(i);
		}

		else if (projection_secondary > 0 && std::abs(point.z - maxZ) < THRESH)
		{
			indices_upper->indices.push_back(i);
		}
	}

	// Extract point clouds for each part
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_upper(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lower(new pcl::PointCloud<pcl::PointXYZ>);

	extract.setIndices(indices_left);
	extract.filter(*cloud_left);

	extract.setIndices(indices_right);
	extract.filter(*cloud_right);

	extract.setIndices(indices_upper);
	extract.filter(*cloud_upper);

	extract.setIndices(indices_lower);
	extract.filter(*cloud_lower);

	std::cout << "Total size:" << cloud->points.size() << std::endl;
	// Output results
	std::cout << "Cloud split into 4 parts:" << std::endl;
	std::cout << "Left Upper size: " << cloud_left->points.size() << std::endl;
	std::cout << "Right Upper size: " << cloud_right->points.size() << std::endl;
	std::cout << "Left Lower size: " << cloud_upper->points.size() << std::endl;
	std::cout << "Right Lower size: " << cloud_lower->points.size() << std::endl;

	int nSize = cloud_left->points.size() + cloud_right->points.size() + cloud_upper->points.size() + cloud_lower->points.size();
	std::cout << nSize << std::endl;
	// ±£´æ½á¹û
	io::savePCDFile("edge1.pcd", *cloud_left);
	io::savePCDFile("edge2.pcd", *cloud_right);
	io::savePCDFile("edge3.pcd", *cloud_upper);
	io::savePCDFile("edge4.pcd", *cloud_lower);
}

int main(int argc, char** argv) 
{
	// Load your point cloud here (for example purposes, loading a PCD file)
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("rect1.pcd", *cloud) == -1) 
	{
		PCL_ERROR("Couldn't read the file input.pcd\n");
		return -1;
	}

	SplitPointCloud(cloud);

	return 0;
}