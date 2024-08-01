#include <iostream>
#include <vector>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>

void visualizePointCloud(const std::string& window_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::visualization::PCLVisualizer viewer(window_name);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}
}

void visualizePlane(const std::string& window_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::ModelCoefficients plane)
{
	pcl::visualization::PCLVisualizer viewer(window_name);
	viewer.addPointCloud(cloud, "cloud");
	viewer.addPlane(plane, "Plane");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}
}

void visualizePointCloudWithPlaneAndBox(const std::string& window_name,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	const Eigen::Vector4f& plane_coefficients,
	const Eigen::Vector3f& min_pt,
	const Eigen::Vector3f& max_pt)
{
	pcl::visualization::PCLVisualizer viewer(window_name);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	// Visualize plane
	Eigen::Vector3f normal(plane_coefficients[0], plane_coefficients[1], plane_coefficients[2]);
	Eigen::Vector3f point_on_plane(-plane_coefficients[3] * normal[0],
		-plane_coefficients[3] * normal[1],
		-plane_coefficients[3] * normal[2]);

	float plane_size = 5.0f; // Size of the visualized plane
	Eigen::Vector3f u = normal.cross(Eigen::Vector3f(0, 0, 1)).normalized();
	Eigen::Vector3f v = normal.cross(u).normalized();

	// Define plane corners
	Eigen::Vector3f p1 = point_on_plane + u * plane_size - v * plane_size;
	Eigen::Vector3f p2 = point_on_plane - u * plane_size - v * plane_size;
	Eigen::Vector3f p3 = point_on_plane - u * plane_size + v * plane_size;
	Eigen::Vector3f p4 = point_on_plane + u * plane_size + v * plane_size;

	// Add plane as polygon
	pcl::PolygonMesh mesh;
	pcl::Vertices vertices;
	vertices.vertices.push_back(0);
	vertices.vertices.push_back(1);
	vertices.vertices.push_back(2);
	vertices.vertices.push_back(3);
	mesh.polygons.push_back(vertices);

	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	plane_cloud->points.push_back(pcl::PointXYZ(p1[0], p1[1], p1[2]));
	plane_cloud->points.push_back(pcl::PointXYZ(p2[0], p2[1], p2[2]));
	plane_cloud->points.push_back(pcl::PointXYZ(p3[0], p3[1], p3[2]));
	plane_cloud->points.push_back(pcl::PointXYZ(p4[0], p4[1], p4[2]));

	pcl::toPCLPointCloud2(*plane_cloud, mesh.cloud);

	viewer.addPolygonMesh(mesh, "plane");

	// Visualize bounding box
	pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	box_cloud->points.push_back(pcl::PointXYZ(min_pt[0], min_pt[1], min_pt[2]));
	box_cloud->points.push_back(pcl::PointXYZ(max_pt[0], min_pt[1], min_pt[2]));
	box_cloud->points.push_back(pcl::PointXYZ(max_pt[0], max_pt[1], min_pt[2]));
	box_cloud->points.push_back(pcl::PointXYZ(min_pt[0], max_pt[1], min_pt[2]));
	box_cloud->points.push_back(pcl::PointXYZ(min_pt[0], min_pt[1], max_pt[2]));
	box_cloud->points.push_back(pcl::PointXYZ(max_pt[0], min_pt[1], max_pt[2]));
	box_cloud->points.push_back(pcl::PointXYZ(max_pt[0], max_pt[1], max_pt[2]));
	box_cloud->points.push_back(pcl::PointXYZ(min_pt[0], max_pt[1], max_pt[2]));

	viewer.addPointCloud<pcl::PointXYZ>(box_cloud, "box_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "box_cloud");

	// Draw lines for bounding box
	viewer.addLine<pcl::PointXYZ>(box_cloud->points[0], box_cloud->points[1], "line1");
	viewer.addLine<pcl::PointXYZ>(box_cloud->points[1], box_cloud->points[2], "line2");
	viewer.addLine<pcl::PointXYZ>(box_cloud->points[2], box_cloud->points[3], "line3");
	viewer.addLine<pcl::PointXYZ>(box_cloud->points[3], box_cloud->points[0], "line4");

	viewer.addLine<pcl::PointXYZ>(box_cloud->points[4], box_cloud->points[5], "line5");
	viewer.addLine<pcl::PointXYZ>(box_cloud->points[5], box_cloud->points[6], "line6");
	viewer.addLine<pcl::PointXYZ>(box_cloud->points[6], box_cloud->points[7], "line7");
	viewer.addLine<pcl::PointXYZ>(box_cloud->points[7], box_cloud->points[4], "line8");

	viewer.addLine<pcl::PointXYZ>(box_cloud->points[0], box_cloud->points[4], "line9");
	viewer.addLine<pcl::PointXYZ>(box_cloud->points[1], box_cloud->points[5], "line10");
	viewer.addLine<pcl::PointXYZ>(box_cloud->points[2], box_cloud->points[6], "line11");
	viewer.addLine<pcl::PointXYZ>(box_cloud->points[3], box_cloud->points[7], "line12");

	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}
}

bool LeastSquareExecuteImpl(Eigen::Vector4d& coeff, const pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudIn)
{
	Eigen::Vector4d centroid;
	Eigen::Matrix3d covarianceMx;

	// 计算归一化协方差矩阵和质心
	pcl::computeMeanAndCovarianceMatrix(*pCloudIn, covarianceMx, centroid);

	// 计算协方差矩阵的特征值与特征向量
	Eigen::Matrix3d eigenVectors;
	Eigen::Vector3d eigenValues;
	pcl::eigen33(covarianceMx, eigenVectors, eigenValues);

	// 查找最小特征值的位置
	Eigen::Vector3d::Index minRow, minCol;
	eigenValues.minCoeff(&minRow, &minCol);

	// 获取平面方程：AX+BY+CZ+D = 0的系数
	Eigen::Vector3d normal = eigenVectors.col(minCol);
	const double crossNorm = normal.stableNorm();
	if (crossNorm < Eigen::NumTraits<double>::dummy_precision())
	{
		return false;
	}

	double D = -normal.dot(centroid.head<3>());

	coeff << normal[0], normal[1], normal[2], D;
	return (coeff[0] != 0 || coeff[1] != 0 || coeff[2] != 0);
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud) == -1)		// sac_plane_test.pcd | Scan_0511_1713.pcd
	{
		PCL_ERROR("点云读取失败 \n");
		return (-1);
	}

	std::cout << "Loaded point cloud with " << cloud->size() << " points." << std::endl;

	// Visualize original point cloud
	//visualizePointCloud("Original Point Cloud", cloud);

	// Define the plane
	Eigen::Vector4d plane_coefficients;
	LeastSquareExecuteImpl(plane_coefficients, cloud);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

	// 将 Eigen::Vector4d 的值转换为 pcl::ModelCoefficients
	coefficients->values.push_back(static_cast<float>(plane_coefficients[0]));  // A
	coefficients->values.push_back(static_cast<float>(plane_coefficients[1]));  // B
	coefficients->values.push_back(static_cast<float>(plane_coefficients[2]));  // C
	coefficients->values.push_back(static_cast<float>(plane_coefficients[3]));  // D

	// Visualize original point cloud
	visualizePlane("FittedPlane", cloud, *coefficients);

	//Eigen::Vector3f normal(plane_coefficients[0], plane_coefficients[1], plane_coefficients[2]);
	//Eigen::Vector3f point_on_plane(-plane_coefficients[3] * normal[0],
	//	-plane_coefficients[3] * normal[1],
	//	-plane_coefficients[3] * normal[2]);

	//// Calculate bounding box vertices
	//float distance_threshold = 5.0f; // Increased threshold for testing
	//Eigen::Vector3f up(0, 0, 1);
	//Eigen::Vector3f u = normal.cross(up).normalized();
	//Eigen::Vector3f v = normal.cross(u).normalized();

	//Eigen::Vector3f min_pt = point_on_plane - u * distance_threshold / 2.0f - v * distance_threshold / 2.0f;
	//Eigen::Vector3f max_pt = point_on_plane + u * distance_threshold / 2.0f + v * distance_threshold / 2.0f;

	//std::cout << "CropBox min: " << min_pt.transpose() << std::endl;
	//std::cout << "CropBox max: " << max_pt.transpose() << std::endl;

	//// Crop the point cloud
	//pcl::CropBox<pcl::PointXYZ> crop_box;
	//crop_box.setInputCloud(cloud);
	//crop_box.setMin(Eigen::Vector4f(min_pt[0], min_pt[1], min_pt[2], 1.0));
	//crop_box.setMax(Eigen::Vector4f(max_pt[0], max_pt[1], max_pt[2], 1.0));

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>);
	//crop_box.filter(*cloud_cropped);

	//std::cout << "Cropped point cloud with " << cloud_cropped->size() << " points." << std::endl;

	//if (cloud_cropped->empty()) {
	//	std::cerr << "Cropped point cloud is empty. Please check the bounding box parameters." << std::endl;
	//	return -1;
	//}

	//// Visualize point cloud with plane and bounding box
	//visualizePointCloudWithPlaneAndBox("Point Cloud with Plane and Box", cloud, plane_coefficients, min_pt, max_pt);

	//// Project the cropped point cloud onto the plane
	//pcl::ProjectInliers<pcl::PointXYZ> proj;
	//proj.setInputCloud(cloud_cropped);
	//proj.setModelType(pcl::SACMODEL_PLANE);
	//proj.setModelCoefficients(coefficients);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	//proj.filter(*cloud_projected);

	//std::cout << "Projected point cloud with " << cloud_projected->size() << " points." << std::endl;

	//// Visualize projected point cloud
	//visualizePointCloud("Projected Point Cloud", cloud_projected);

	//// Extract concave hull
	//pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
	//concave_hull.setInputCloud(cloud_projected);
	//concave_hull.setAlpha(0.01); // Adjust alpha as needed

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	//concave_hull.reconstruct(*cloud_hull);

	//std::cout << "Concave hull with " << cloud_hull->size() << " points." << std::endl;

	return 0;
}