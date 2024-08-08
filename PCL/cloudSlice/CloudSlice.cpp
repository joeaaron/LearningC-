#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <omp.h> // 引入OpenMP支持
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/concave_hull.h>

#define ENABLE_DISPLAY 1		// 定义一个宏，用于控制显示状态

inline Eigen::Vector4d CalcPlane(const pcl::PointXYZ& a, const pcl::PointXYZ& b, const pcl::PointXYZ& c)
{
	Eigen::Vector4d param;
	Eigen::Vector3d p1, p2, p3, p1p2, p1p3, N, N1;
	p1 << a.x, a.y, a.z;
	p2 << b.x, b.y, b.z;
	p3 << c.x, c.y, c.z;
	p1p2 = p1 - p2;
	p1p3 = p3 - p1;

	N = p1p2.cross(p1p3);
	N1 = N / N.norm();

	param[0] = N1[0];
	param[1] = N1[1];
	param[2] = N1[2];
	param[3] = -N1.dot(p1);

	return param;
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("example.pcd", *cloud) == -1)		// sac_plane_test.pcd | Scan_0511_1713.pcd
	{
		PCL_ERROR("点云读取失败 \n");
		return (-1);
	}

	//pcl::PointXYZ pa = {-0.089717, 0.118919, 0.045746};
	//pcl::PointXYZ pb = {0.026236, 0.122255, 0.022206};
	//pcl::PointXYZ pc = {0.058940, 0.056647, 0.020176};

	//Eigen::Vector4d n;
	//n = CalcPlane(pa, pb, pc);

	//double delta = 0.001;  // 设置切片的0.5倍厚度,厚度和点云密度相关
	//std::vector<int> pointIdx;

	//for (int i = 0; i < cloud->size(); ++i)
	//{
	//	double wr = n[0] * (*cloud)[i].x + n[1] * (*cloud)[i].y + n[2] * (*cloud)[i].z + n[3] - delta;
	//	double wl = n[0] * (*cloud)[i].x + n[1] * (*cloud)[i].y + n[2] * (*cloud)[i].z + n[3] + delta;
	//	if (wr * wl <= 0)
	//	{
	//		pointIdx.emplace_back(i);
	//	}
 //	}

	//pcl::PointCloud<pcl::PointXYZ>::Ptr sliceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::copyPointCloud(*cloud, pointIdx, *sliceCloud);

	//// 切片点投影到平面
	//pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);

	//for (size_t i = 0; i < sliceCloud->points.size(); ++i) {
	//	// 获取当前点的坐标
	//	float x = sliceCloud->points[i].x;
	//	float y = sliceCloud->points[i].y;
	//	float z = sliceCloud->points[i].z;

	//	// 计算点到平面的垂直距离
	//	float distance = abs(n[0] * x + n[1] * y + n[2] * z - n[3]) / sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);

	//	// 计算投影点的坐标
	//	float xp = x - distance * n[0] / sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
	//	float yp = y - distance * n[1] / sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
	//	float zp = z - distance * n[2] / sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);

	//	// 将投影点加入新的点云中
	//	projectedCloud->points.push_back(pcl::PointXYZ(xp, yp, zp));
	//}

	// Extract concave hull
	pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
	concave_hull.setInputCloud(cloud);
	concave_hull.setAlpha(5);					// Adjust alpha as needed

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PolygonMesh mesh;
	concave_hull.reconstruct(mesh);

#if ENABLE_DISPLAY
	//-----------------结果显示---------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cloud boundary extraction - AS"));

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v1);
	viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
	viewer->setBackgroundColor(0.5, 0.5, 0.5, v2);
	viewer->addText("Sliced point clouds", 10, 10, "v2_text", v2);

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "raw cloud", v1);
	//viewer->addPointCloud<pcl::PointXYZ>(cloud_hull, "sliced cloud", v2);
	viewer->addPolygonMesh(mesh, "mesh cloud", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "raw cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "sliced cloud", v2);

	//viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(1000);
	}
#endif

	return 0;
}