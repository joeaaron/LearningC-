#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <omp.h> // 引入OpenMP支持
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

#define ENABLE_DISPLAY 1		// 定义一个宏，用于控制显示状态

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

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

pcl::PointCloud<pcl::PointXYZ>::Ptr TransformCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, Eigen::Matrix4d transformT)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZ>);
	cloudout->width = cloudIn->width;
	cloudout->height = cloudIn->height;
	cloudout->is_dense = cloudIn->is_dense;
	cloudout->points.resize(cloudIn->points.size());

	// 启用OpenMP并行化
#pragma omp parallel for 
	// 遍历原始点云中的每个点  
	for (int i = 0; i < cloudIn->points.size(); ++i)
	{
		// 创建一个4x1的齐次坐标向量（x, y, z, 1）  
		Eigen::Vector4d point_homogeneous(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z, 1.0);

		// 应用变换矩阵到齐次坐标向量  
		Eigen::Vector4d transformed_point_homogeneous = transformT * point_homogeneous;

		// 提取变换后的3D坐标（x, y, z）  
		cloudout->points[i].x = transformed_point_homogeneous(0);
		cloudout->points[i].y = transformed_point_homogeneous(1);
		cloudout->points[i].z = transformed_point_homogeneous(2);
	}

	return cloudout;
}

inline Eigen::Vector4d CalcPlane(Eigen::Vector3d center, Eigen::Vector3d normal)
{
	Eigen::Vector4d planeEquation;

	// 计算平面方程中的 d
	double d = normal.dot(center);
	planeEquation << normal, -d;

	return planeEquation;
}

void GetSlice(pcl::PointCloud<pcl::PointXYZ>::Ptr& sliceCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Eigen::Vector4d& n)
{
	double delta = 2;			// 设置切片的0.5倍厚度,厚度和点云密度相关
	std::vector<int> pointIdx;

	for (int i = 0; i < cloud->size(); ++i)
	{
		double wr = n[0] * (*cloud)[i].x + n[1] * (*cloud)[i].y + n[2] * (*cloud)[i].z + n[3] - delta;
		double wl = n[0] * (*cloud)[i].x + n[1] * (*cloud)[i].y + n[2] * (*cloud)[i].z + n[3] + delta;
		if (wr * wl <= 0)
		{
			pointIdx.emplace_back(i);
		}
	}

	pcl::copyPointCloud(*cloud, pointIdx, *sliceCloud);
	// 调试用
	//pcl::PCDWriter writer;
	//writer.write("sliceCloud.pcd", *sliceCloud, false);
	// ****************************包围盒内点云显示******************************
#if ENABLE_DISPLAY
	pcl::visualization::PCLVisualizer viewer("CropBox Viewer");
	viewer.addPointCloud(sliceCloud);
	viewer.resetCamera();
	// 等待直到视图关闭
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
#endif
}

void ProjMethod(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	// 任意平面
	Eigen::Vector3d center(4.734, 9.470, 1.706);
	Eigen::Vector3d n(0.267, 0.535, 0.802);
	Eigen::Vector3d zDir(0, 0, 1);

	Eigen::Vector3d axis = n.cross(zDir);
	double angle = acos(n.dot(zDir) / n.norm());

	Eigen::AngleAxisd rotation(angle, axis.normalized());
	Eigen::Matrix3d rotationMatrix = rotation.toRotationMatrix();

	// 使用旋转矩阵旋转点云
	Eigen::Matrix4d transMtx = Eigen::Matrix4d::Identity();
	transMtx.block<3, 3>(0, 0) = rotationMatrix;

	pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	rotatedCloud = TransformCloud(cloud, transMtx);
	Eigen::Vector3d rotatedPt = rotationMatrix * center;

	// 得到包围盒内点云
	Eigen::Vector4d plane = CalcPlane(rotatedPt, zDir);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sliceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	GetSlice(sliceCloud, rotatedCloud, plane);

	// 切片点投影到平面
	pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	double d = -(zDir[0] * rotatedPt(0) + zDir[1] * rotatedPt(1) + zDir[2] * rotatedPt(2));
	for (size_t i = 0; i < sliceCloud->points.size(); ++i) {
		// 获取当前点的坐标
		float x = sliceCloud->points[i].x;
		float y = sliceCloud->points[i].y;
		float z = sliceCloud->points[i].z;

		// 计算点到平面的垂直距离
		float distance = abs(zDir[0] * x + zDir[1] * y + zDir[2] * z - d) / sqrt(zDir[0] * zDir[0] + zDir[1] * zDir[1] + zDir[2] * zDir[2]);

		// 计算投影点的坐标
		float xp = x - distance * zDir[0] / sqrt(n[0] * zDir[0] + zDir[1] * zDir[1] + zDir[2] * zDir[2]);
		float yp = y - distance * zDir[1] / sqrt(n[0] * zDir[0] + zDir[1] * zDir[1] + zDir[2] * zDir[2]);
		float zp = z - distance * zDir[2] / sqrt(n[0] * zDir[0] + zDir[1] * zDir[1] + zDir[2] * zDir[2]);

		// 将投影点加入新的点云中
		projectedCloud->points.push_back(pcl::PointXYZ(xp, yp, zp));
	}

	projectedCloud->width = projectedCloud->points.size();
	projectedCloud->height = 1;
	projectedCloud->is_dense = false;

	pcl::PCDWriter writer;
	writer.write("project.pcd", *projectedCloud, false);

	// Extract concave hull
	pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
	concave_hull.setInputCloud(projectedCloud);
	concave_hull.setAlpha(8.5);					// Adjust alpha as needed

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PolygonMesh mesh;
	concave_hull.reconstruct(*cloud_hull);

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
	viewer->addPointCloud<pcl::PointXYZ>(cloud_hull, "sliced cloud", v2);
	//viewer->addPolygonMesh(mesh, "mesh cloud", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "raw cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "sliced cloud", v2);

	//viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->resetCamera();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(1000);
	}
#endif
	/*
	// ***********************三维凸包算法*************************

	pcl::ConvexHull<PointT> convexHull;
	convexHull.setInputCloud(cloud);
	convexHull.setDimension(3);
	convexHull.setComputeAreaVolume(true);
	//保存凸包中的面要素
	std::vector<pcl::Vertices> polygons;
	pcl::PointCloud<PointT>::Ptr surface_hull(new pcl::PointCloud<PointT>);
	convexHull.reconstruct(*surface_hull, polygons);

	double v = convexHull.getTotalVolume();

	// ****************************可视化******************************
	auto viewportsVis = [](pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudS,
		pcl::PointCloud<PointT>::Ptr surface_hull,
		std::vector<pcl::Vertices> polygons)
		{
			// 创建3D窗口并添加显示点云其包括法线
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorS(cloudS, 0, 200, 100);

			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerVGF(new pcl::visualization::PCLVisualizer("3D Viewer"));
			viewerVGF->setBackgroundColor(0, 0, 0);
			//viewerVGF->addPointCloud<pcl::PointXYZ>(cloudS, colorS, "source cloud");
			//viewerVGF->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "source cloud");
			viewerVGF->addPolygonMesh<PointT>(surface_hull, polygons, "polyline");
			viewerVGF->addLine(cloudS->points[cloudS->size() - 1], cloudS->points[0], "line" + std::to_string(cloudS->size() - 1));

			viewerVGF->spin();
		};

	viewportsVis(cloud, surface_hull, polygons);
	*/
}

// Step 1: 计算点云密度
double ComputeDensity(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int N, int m) 
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	std::vector<double> vDensities;
	for (int i = 0; i < N; ++i) {
		std::vector<int> pointIdxNKNSearch(m);
		std::vector<float> pointNKNSquaredDistance(m);

		pcl::PointXYZ searchPoint = (*cloud)[rand() % cloud->points.size()];

		if (kdtree.nearestKSearch(searchPoint, m, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			double density = 0.0;
			for (double dist : pointNKNSquaredDistance)
				density += sqrt(dist);
			vDensities.push_back(density / m);
		}
	}
	return std::accumulate(vDensities.begin(), vDensities.end(), 0.0) / vDensities.size();

	//// 输出密度信息或做其他处理
	//for (auto density : densities)
	//	std::cout << "Density: " << density << std::endl;
}

// Step 2: 得到左右点云 (假设我们沿着Z轴切片)
void SlicePointCloud(
	pcl::PointCloud<pcl::PointXYZ>::Ptr lpoints,
	pcl::PointCloud<pcl::PointXYZ>::Ptr rpoints,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	double dPos, double delta)
{
	for (const auto& point : cloud->points) 
	{
		if (point.z >= dPos - delta / 2 && point.z <= dPos)
		{
			lpoints->points.push_back(point);
		}
		else if (point.z > dPos && point.z <= dPos + delta / 2)
		{
			rpoints->points.push_back(point);
		}
	}

	std::cout << "Lpoints: " << lpoints->points.size() << ", Rpoints: " << rpoints->points.size() << std::endl;
}

// Step 3: 匹配 lpoint 和 rpoint 并计算交点
void FindCorres(pcl::PointCloud<pcl::PointXYZ>::Ptr sliceCloud,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr lpoints, 
	const pcl::PointCloud<pcl::PointXYZ>::Ptr rpoints, 
	double dPos, double threshold)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(rpoints);

	for (const auto& lpoint : lpoints->points) 
	{
		// 平面上的点直接加进去
		if (std::abs(lpoint.z - dPos) < 1e-4)
		{
			sliceCloud->points.push_back(lpoint);
			continue;
		}

		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);

		if (kdtree.nearestKSearch(lpoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			float dist = sqrt(pointNKNSquaredDistance[0]);
			if (dist < threshold) 
			{
				pcl::PointXYZ rpoint = rpoints->points[pointIdxNKNSearch[0]];
				pcl::PointXYZ intersection;

				double dCoef1 = dPos - lpoint.z;
				double dCoef2 = rpoint.z - lpoint.z;

				if (fabs(dCoef2) < DBL_EPSILON) return;

				intersection.x = dCoef1 * (rpoint.x - lpoint.x) / dCoef2 + lpoint.x;
				intersection.y = dCoef1 * (rpoint.y - lpoint.y) / dCoef2 + lpoint.y;
				intersection.z = dPos;

				sliceCloud->points.push_back(intersection);
			}
		}
	}

#if ENABLE_DISPLAY
	pcl::visualization::PCLVisualizer viewer("Viewer");
	viewer.addPointCloud(sliceCloud);
	viewer.resetCamera();

	// 等待直到视图关闭
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
#endif

	std::cout << "Intersection points: " << sliceCloud->points.size() << std::endl;
}

void IntersectMethodZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	// Step 1: 计算点云密度
	double dDelta = ComputeDensity(cloud, 100, 5);
	double dPos = -17.7557755;

	// Step 2: 得到左右点云 (假设我们沿着Z轴切片)
	pcl::PointCloud<pcl::PointXYZ>::Ptr lpoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr rpoints(new pcl::PointCloud<pcl::PointXYZ>);
	SlicePointCloud(lpoints, rpoints, cloud, dPos, 10 * dDelta);

//#if ENABLE_DISPLAY
//	pcl::visualization::PCLVisualizer viewer("Viewer");
//	viewer.addPointCloud(rpoints);
//	viewer.resetCamera();
//
//	// 等待直到视图关闭
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce(100);
//	}
//#endif

	// Step 3: 匹配 lpoint 和 rpoint 并计算交点
	//FindCorres(lpoints, rpoints, dPos, 8 * dDelta);
}

void IntersectMethod(pcl::PointCloud<pcl::PointXYZ>::Ptr sliceCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	// TEST CASE 01
	//Eigen::Vector3d center(-25.402, -14.551, -19.163); 
	//Eigen::Vector3d n(0.577, 0.577, 0.577);

	// TEST CASE 02
	Eigen::Vector3d center(-95.978, 45.836, 0.005);
	Eigen::Vector3d n(1, 3, 3);

	// TEST CASE 03
	//Eigen::Vector3d center(95.665, 72.443, -16.049);
	//Eigen::Vector3d n(10, 12, 20);

	// TEST CASE 04
	//Eigen::Vector3d center(-0.001, 33.146, -12.500);
	//Eigen::Vector3d n(0.267, 0.535, 0.802);

	Eigen::Vector3d z(0, 0, 1);
	Eigen::Vector3d axis = n.normalized().cross(z);
	double angle = acos(n.dot(z) / n.norm());

	Eigen::AngleAxisd rotation(angle, axis.normalized());
	Eigen::Matrix3d rotationMatrix = rotation.toRotationMatrix();

	// 使用旋转矩阵旋转点云
	Eigen::Matrix4d transMtx = Eigen::Matrix4d::Identity();
	transMtx.block<3, 3>(0, 0) = rotationMatrix;

	pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud(new pcl::PointCloud<pcl::PointXYZ>);
	transCloud = TransformCloud(cloud, transMtx);
	Eigen::Vector3d rotatedPt = rotationMatrix * center;

	// Step 1: 计算点云密度
	double dDelta = ComputeDensity(transCloud, 100, 5);

	// Step 2: 得到左右点云 (假设我们沿着Z轴切片)
	pcl::PointCloud<pcl::PointXYZ>::Ptr lpoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr rpoints(new pcl::PointCloud<pcl::PointXYZ>);

	double dMaxDisToPlane = 2.000;  // 最大点至平面距离
	SlicePointCloud(lpoints, rpoints, transCloud, rotatedPt[2], dMaxDisToPlane/* 8 * dDelta*/);

//#if ENABLE_DISPLAY
//	pcl::visualization::PCLVisualizer viewer("Viewer");
//	viewer.addPointCloud(rpoints);
//	viewer.resetCamera();
//
//	// 等待直到视图关闭
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce(100);
//	}
//#endif

	// Step 3: 匹配 lpoint 和 rpoint 并计算交点
	FindCorres(sliceCloud, lpoints, rpoints, rotatedPt[2], 2 * dDelta);
}

std::tuple<double, double> CalAvgStd(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	size_t numPoints = cloud->points.size();

	if (numPoints < 2) 
	{
		return std::make_tuple(NAN, NAN);
		std::cerr << "Not enough points to compute distances." << std::endl;
	}

	std::vector<double> vDis;
	for (int i = 0; i < numPoints; ++i)
	{
		size_t nextIndex = (i + 1) % numPoints; // 最后一个点与第一个点相连
		const pcl::PointXYZ& pt1 = cloud->points[i];
		const pcl::PointXYZ& pt2 = cloud->points[nextIndex];
		auto distance = pcl::euclideanDistance(pt1, pt2);
		vDis.emplace_back(distance);
		
	}
	double dAvg = std::accumulate(vDis.begin(), vDis.end(), 0.0) / vDis.size();
	double dSqu = std::inner_product(vDis.begin(), vDis.end(), vDis.begin(), 0.0f);
	double dStd = std::sqrt(dSqu / vDis.size() - dAvg * dAvg);

	return std::make_tuple(dAvg, dStd);
}

double CalculateAngle(const pcl::PointXYZ& origin, const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) 
{
	double angle1 = atan2(p1.y - origin.y, p1.x - origin.x);
	double angle2 = atan2(p2.y - origin.y, p2.x - origin.x);
	double angle = angle2 - angle1;

	// 确保角度在 [0, 2 * PI] 范围内
	if (angle < 0) angle += 2 * M_PI;
	return angle;
}

PointT FindStartPt(const PointCloud::Ptr& cloud)
{
	PointT startPt = cloud->points[0];
	for (const auto& pt : cloud->points)
	{
		if (pt.x < startPt.x || (pt.x == startPt.x && pt.y < startPt.y))
		{
			startPt = pt;
		}
	}

	return startPt;
}

bool IsLineIntersected(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3, const pcl::PointXYZ& p4) 
{
	// 计算向量叉乘
	auto crossProduct = [](double x1, double y1, double x2, double y2) {
		return x1 * y2 - y1 * x2;
		};

	double v1_x = p2.x - p1.x;
	double v1_y = p2.y - p1.y;
	double v2_x = p4.x - p3.x;
	double v2_y = p4.y - p3.y;

	double cross1 = crossProduct(v1_x, v1_y, p3.x - p1.x, p3.y - p1.y);
	double cross2 = crossProduct(v1_x, v1_y, p4.x - p1.x, p4.y - p1.y);

	double cross3 = crossProduct(v2_x, v2_y, p1.x - p3.x, p1.y - p3.y);
	double cross4 = crossProduct(v2_x, v2_y, p2.x - p3.x, p2.y - p3.y);

	// 检查叉乘是否改变符号，判断线段是否相交
	return (cross1 * cross2 < 0) && (cross3 * cross4 < 0);
}

bool DoLinesIntersect(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3, const pcl::PointXYZ& p4) {
	auto cross = [](double x1, double y1, double x2, double y2) {
		return x1 * y2 - y1 * x2;
		};

	double dx1 = p2.x - p1.x;
	double dy1 = p2.y - p1.y;
	double dx2 = p4.x - p3.x;
	double dy2 = p4.y - p3.y;

	double delta = cross(dx1, dy1, dx2, dy2);
	if (delta == 0) {
		return false; // 平行，不相交
	}

	double s = cross(p3.x - p1.x, p3.y - p1.y, dx2, dy2) / delta;
	double t = cross(p3.x - p1.x, p3.y - p1.y, dx1, dy1) / delta;

	return s >= 0 && s <= 1 && t >= 0 && t <= 1; // 是否在线段内相交
}

bool IsSelfIntersecting(const PointCloud::Ptr& sortedPoints, const pcl::PointXYZ& newPoint) 
{
	size_t last = sortedPoints->points.size() - 1;

	for (size_t i = 0; i < last - 1; ++i) {
		const pcl::PointXYZ& p1 = sortedPoints->points[i];
		const pcl::PointXYZ& p2 = sortedPoints->points[i + 1];
		const pcl::PointXYZ& lastPoint = sortedPoints->points[last];

		// 检查新形成的线段 (lastPoint, newPoint) 是否与现有的线段 (p1, p2) 相交
		if (DoLinesIntersect(p1, p2, lastPoint, newPoint)) {
			return true; // 发生自交
		}
	}
	return false; // 没有自交
}

// Function to calculate the angle between two vectors
double calculateAngle(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3) {
	// Calculate vectors
	double v1_x = p2.x - p1.x;
	double v1_y = p2.y - p1.y;
	double v2_x = p3.x - p2.x;
	double v2_y = p3.y - p2.y;

	// Calculate angles using atan2
	double angle1 = std::atan2(v1_y, v1_x);
	double angle2 = std::atan2(v2_y, v2_x);

	// Calculate the difference and normalize to [0, 360) degrees
	double angle = std::fmod((angle2 - angle1) * 180.0 / M_PI + 360, 360.0);
	if (angle > 180) angle = 360 - angle; // Convert to [0, 180]

	return angle;
}

// Function to check if two line segments (p1-p2 and p3-p4) intersect
bool segmentsIntersect(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3, const pcl::PointXYZ& p4) {
	auto crossProduct = [](const pcl::PointXYZ& p, const pcl::PointXYZ& q, const pcl::PointXYZ& r) {
		return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
		};

	// Check if the line segments p1-p2 and p3-p4 intersect
	double d1 = crossProduct(p3, p4, p1);
	double d2 = crossProduct(p3, p4, p2);
	double d3 = crossProduct(p1, p2, p3);
	double d4 = crossProduct(p1, p2, p4);

	// If the signs of d1 and d2 differ, and the signs of d3 and d4 differ, the segments intersect
	if ((d1 * d2 < 0) && (d3 * d4 < 0)) {
		return true;
	}

	return false;
}

// Function to check if adding a point will cause intersection with existing segments
bool willCauseIntersection(const PointCloud::Ptr& ordered_points, const pcl::PointXYZ& new_point, size_t insert_position) {
	const pcl::PointXYZ& prev_point = ordered_points->points[insert_position - 1];
	const pcl::PointXYZ& next_point = ordered_points->points[insert_position];

	// Check for intersection with other segments
	for (size_t i = 0; i < ordered_points->points.size() - 1; ++i) {
		const pcl::PointXYZ& p1 = ordered_points->points[i];
		const pcl::PointXYZ& p2 = ordered_points->points[i + 1];

		// Skip the segments that involve the points where we are inserting the new point
		if ((p1.x == prev_point.x && p1.y == prev_point.y && p2.x == next_point.x && p2.y == next_point.y) ||
			(p1.x == next_point.x && p1.y == next_point.y && p2.x == prev_point.x && p2.y == prev_point.y)) {
			continue;
		}

		if (segmentsIntersect(p1, p2, prev_point, new_point) || segmentsIntersect(p1, p2, new_point, next_point)) {
			return true; // Insertion would cause an intersection
		}
	}

	return false; // No intersection detected
}

PointCloud::Ptr SortPointsUsingHull(const PointCloud::Ptr& cloud)
{
	// Step 1: Compute Convex Hull
	PointCloud::Ptr convex_hull_cloud(new PointCloud());
	pcl::ConvexHull<pcl::PointXYZ> convex_hull;
	convex_hull.setInputCloud(cloud);
	convex_hull.reconstruct(*convex_hull_cloud);

	// Step 2: Compute Concave Hull
	PointCloud::Ptr concave_hull_cloud(new PointCloud());
	pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
	concave_hull.setInputCloud(cloud);
	concave_hull.setAlpha(8.5); // Alpha value controls concavity
	concave_hull.reconstruct(*concave_hull_cloud);

	PointCloud::Ptr sortedPoints(new PointCloud);

	// Add convex hull points first (they form the outer boundary)
	for (const auto& point : convex_hull_cloud->points) {
		sortedPoints->points.push_back(point);
	}

	// Insert concave hull points into the ordered list based on proximity to the convex hull
	for (const auto& point : concave_hull_cloud->points) {
		// Find the closest point in the ordered convex hull points
		double min_distance = std::numeric_limits<double>::max();
		size_t insert_position = 0;

		for (size_t i = 0; i < sortedPoints->points.size(); ++i) {
			double distance = std::sqrt(std::pow(point.x - sortedPoints->points[i].x, 2) +
				std::pow(point.y - sortedPoints->points[i].y, 2) +
				std::pow(point.z - sortedPoints->points[i].z, 2));
			if (distance < min_distance) {
				min_distance = distance;
				insert_position = i;
			}
		}

		// Insert the concave point near its closest convex point
		sortedPoints->points.insert(sortedPoints->points.begin() + insert_position, point);
	}

	return sortedPoints;
}

PointCloud::Ptr SortPointsUsingHullEx(const PointCloud::Ptr& cloud)
{
	// Step 1: Compute Convex Hull
	PointCloud::Ptr convex_hull_cloud(new PointCloud());
	pcl::ConvexHull<pcl::PointXYZ> convex_hull;
	convex_hull.setInputCloud(cloud);
	convex_hull.reconstruct(*convex_hull_cloud);

	// Step 2: Compute Concave Hull
	PointCloud::Ptr concave_hull_cloud(new PointCloud());
	pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
	concave_hull.setInputCloud(cloud);
	concave_hull.setAlpha(8.5); // Alpha value controls concavity
	concave_hull.reconstruct(*concave_hull_cloud);

	PointCloud::Ptr sortedPoints(new PointCloud);
	// Add convex hull points first (they form the outer boundary)
	for (const auto& point : convex_hull_cloud->points) {
		sortedPoints->points.push_back(point);
	} 

	// Insert concave hull points into the ordered list based on proximity and angle consistency
	for (const auto& concave_point : concave_hull_cloud->points) 
	{
		double min_distance = std::numeric_limits<double>::max();
		size_t best_insert_position = 0;
		double best_angle_change = std::numeric_limits<double>::max();

		// Find the best insert position based on proximity and minimal angle change
		for (size_t i = 0; i < sortedPoints->points.size() - 1; ++i) {
			const auto& prev_point = sortedPoints->points[i];
			const auto& next_point = sortedPoints->points[i + 1];

			// Compute the distance from the concave point to the segment between prev_point and next_point
			double distance_to_segment = std::sqrt(std::pow(concave_point.x - prev_point.x, 2) +
				std::pow(concave_point.y - prev_point.y, 2));

			// Compute angle change if this point was inserted
			double angle_change = calculateAngle(prev_point, concave_point, next_point);

			// Select this position if the distance is smaller and angle change is more favorable
			if (distance_to_segment < min_distance && angle_change < best_angle_change) {
				min_distance = distance_to_segment;
				best_insert_position = i + 1;
				best_angle_change = angle_change;
			}
		}

		// Before inserting, check for intersection
		if (!willCauseIntersection(sortedPoints, concave_point, best_insert_position)) 
		{
			// Insert the concave point at the best position found
			sortedPoints->points.insert(sortedPoints->points.begin() + best_insert_position, concave_point);
		}
		else 
		{
			std::cout << "Insertion of point at (" << concave_point.x << ", " << concave_point.y << ") would cause intersection. Skipping this point." << std::endl;
		}
	}
	
	return sortedPoints;
}

// 使用 k-d Tree 查找最近点并进行排序
PointCloud::Ptr SortPointsUsingKDTree(const PointCloud::Ptr& cloud)
{
	// 构建 k-d Tree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	// 计算点云密度
	double dAvgDis = 0.0;
	int nNum = 0;
	for (int i = 0; i < cloud->size(); ++i)
	{
		std::vector<int> indiceId;
		std::vector<float> disSquare;

		if (kdtree.nearestKSearch(cloud->points[i], 2, indiceId, disSquare) > 0)
		{
			dAvgDis += sqrt(disSquare[1]);
			nNum++;
		}
	}
	dAvgDis /= nNum;


	// 结果点集
	PointCloud::Ptr sortedPoints(new PointCloud);

	// 选择起始点，例如选择最左下角的点
	pcl::PointXYZ startPoint = cloud->points[0];  // 假设选择第一个点作为起始点
	sortedPoints->points.push_back(startPoint);

	std::vector<bool> used(cloud->points.size(), false);
	used[0] = true;

	pcl::PointXYZ currentPoint = startPoint;

	for (size_t i = 1; i < cloud->points.size(); ++i)
	{
		float searchRadius = dAvgDis;						//0.1f：搜索半径，可根据点云密度调整
		bool foundValidPoint = false;

		while (!foundValidPoint)
		{
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			// kd-tree 半径搜索，控制搜索范围
			if (kdtree.radiusSearch(currentPoint, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= 0)
			{
				searchRadius *= 2;
			}
			else
			{
				int nearestIdx = -1;
				float minDistance = std::numeric_limits<float>::max();

				for (int j = 0; j < pointIdxRadiusSearch.size(); ++j)
				{
					int idx = pointIdxRadiusSearch[j];
					if (!used[idx] && pointRadiusSquaredDistance[j] < minDistance)
					{
						nearestIdx = idx;
						minDistance = pointRadiusSquaredDistance[j];
					}
				}
				// 如果找到合适的最近点
				if (nearestIdx != -1)
				{
					pcl::PointXYZ nearestPoint = cloud->points[nearestIdx];
					sortedPoints->points.push_back(nearestPoint);
					used[nearestIdx] = true;
					currentPoint = nearestPoint;
					foundValidPoint = true;
				}
				else
				{
					searchRadius *= 2;		// 动态调整半径
				}
			}
		}
	}

	return sortedPoints;
}

PointCloud::Ptr SortPointsUsingKDTreeEx(const PointCloud::Ptr& cloud)
{
	// 构建 k-d Tree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	// 计算点云密度
	double dAvgDis = 0.0;
	int nNum = 0;
	for (int i = 0; i < cloud->size(); ++i) {
		std::vector<int> indiceId;
		std::vector<float> disSquare;

		if (kdtree.nearestKSearch(cloud->points[i], 2, indiceId, disSquare) > 0) {
			dAvgDis += sqrt(disSquare[1]);
			nNum++;
		}
	}
	dAvgDis /= nNum;

	// 结果点集
	PointCloud::Ptr sortedPoints(new PointCloud);

	// 选择起始点，例如选择最左下角的点
	pcl::PointXYZ startPoint = FindStartPt(cloud);    // cloud->points[0];  假设选择第一个点作为起始点
	sortedPoints->points.push_back(startPoint);

	std::vector<bool> used(cloud->points.size(), false);
	used[0] = true;

	pcl::PointXYZ currentPoint = startPoint;
	pcl::PointXYZ previousPoint = startPoint;  // 用于计算方向

	for (size_t i = 1; i < cloud->points.size(); ++i) {
		float searchRadius = dAvgDis;
		bool foundValidPoint = false;

		while (!foundValidPoint) {
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			// kd-tree 半径搜索，控制搜索范围
			if (kdtree.radiusSearch(currentPoint, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= 0)
			{
				searchRadius *= 2;
			}
			else
			{
				int bestIdx = -1;
				double minAngle = std::numeric_limits<float>::max();

				for (int j = 0; j < pointIdxRadiusSearch.size(); ++j)
				{
					int idx = pointIdxRadiusSearch[j];
					if (!used[idx])
					{
						double angle = CalculateAngle(previousPoint, currentPoint, cloud->points[idx]);
						if (angle < minAngle)   // 角度变化最小 
						{
							minAngle = angle;
							bestIdx = idx;
						}
					}
				}

				// 如果找到合适的点（按方向排序）
				if (bestIdx != -1) {
					pcl::PointXYZ nextPoint = cloud->points[bestIdx];
					sortedPoints->points.push_back(nextPoint);
					used[bestIdx] = true;
					previousPoint = currentPoint;
					currentPoint = nextPoint;
					foundValidPoint = true;
				}
				else
				{
					searchRadius *= 2;  // 动态调整半径
				}
			}
		}
	}

	return sortedPoints;
}

PointCloud::Ptr SortPointsUsingSelfIntersect(const PointCloud::Ptr& cloud)
{
	// 构建 k-d Tree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	// 计算点云密度
	double dAvgDis = 0.0;
	int nNum = 0;
	for (int i = 0; i < cloud->size(); ++i) {
		std::vector<int> indiceId;
		std::vector<float> disSquare;

		if (kdtree.nearestKSearch(cloud->points[i], 2, indiceId, disSquare) > 0) {
			dAvgDis += sqrt(disSquare[1]);
			nNum++;
		}
	}
	dAvgDis /= nNum;

	// 结果点集
	PointCloud::Ptr sortedPoints(new PointCloud);

	// 选择起始点，例如选择最左下角的点
	pcl::PointXYZ startPoint = FindStartPt(cloud);    // cloud->points[0];  假设选择第一个点作为起始点
	sortedPoints->points.push_back(startPoint);

	std::vector<bool> used(cloud->points.size(), false);
	used[0] = true;

	pcl::PointXYZ currentPoint = startPoint;
	pcl::PointXYZ previousPoint = startPoint;  // 用于计算方向

	for (size_t i = 1; i < cloud->points.size(); ++i) 
	{
		float searchRadius = dAvgDis;
		bool foundValidPoint = false;

		while (!foundValidPoint) 
		{
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			// kd-tree 半径搜索，控制搜索范围
			if (kdtree.radiusSearch(currentPoint, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= 0)
			{
				searchRadius *= 2;
			}
			else
			{
				std::vector<std::pair<double, int>> anglePointPairs;

				for (int j = 0; j < pointIdxRadiusSearch.size(); ++j)
				{
					int idx = pointIdxRadiusSearch[j];
					if (!used[idx])
					{
						double angle = CalculateAngle(previousPoint, currentPoint, cloud->points[idx]);
						anglePointPairs.emplace_back(angle, idx);
					}
				}

				// Sort pairs by angle
				std::sort(anglePointPairs.begin(), anglePointPairs.end());

				// Find the best non-intersecting point
				for (const auto& pair : anglePointPairs) {
					int idx = pair.second;
					pcl::PointXYZ nextPoint = cloud->points[idx];

					if (sortedPoints->points.size() <= 1)
					{
						sortedPoints->points.push_back(nextPoint);
						used[idx] = true;
						previousPoint = currentPoint;
						currentPoint = nextPoint;
						foundValidPoint = true;
						break;
					}

					if (!IsSelfIntersecting(sortedPoints, nextPoint)) {
						sortedPoints->points.push_back(nextPoint);
						used[idx] = true;
						previousPoint = currentPoint;
						currentPoint = nextPoint;
						foundValidPoint = true;
						break;
					}
				}

				if (!foundValidPoint) {
					searchRadius *= 2;  // Increase radius if no valid point is found
				}
			}
		}
	}

	return sortedPoints;
}

void CurveReconstruct1(const PointCloud::Ptr& cloud)
{
	// 计算均值和标准差
	const auto& [mean, stddev] = CalAvgStd(cloud);

	// 欧几里德聚类提取器
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(mean);				// 设置近邻搜索的搜索半径
	ec.setMinClusterSize(3);							// 设置一个聚类需要的最少点数目
	ec.setMaxClusterSize(100000);						// 设置一个聚类需要的最大点数目
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(clusterIndices);

	std::vector<PointCloud::Ptr> vCloud;
	for (const auto& indices : clusterIndices)
	{
		PointCloud::Ptr cloudCluster(new PointCloud);
		for (const auto& index : indices.indices)
			cloudCluster->points.push_back(cloud->points[index]);

		// 进行点排序
		auto startOp = std::chrono::high_resolution_clock::now();
		//PointCloud::Ptr sortedPoints = SortPointsUsingKDTreeEx(cloudCluster);
		PointCloud::Ptr sortedPoints = SortPointsUsingHull(cloudCluster);
		//PointCloud::Ptr sortedPoints = SortPointsUsingHullEx(cloudCluster);
		//PointCloud::Ptr sortedPoints = SortPointsUsingSelfIntersect(cloudCluster);

		// 调试用
		/*pcl::PCDWriter writer;
		sortedPoints->width = sortedPoints->points.size();
		sortedPoints->height = 1;
		sortedPoints->is_dense = false;
		writer.write("SortedCloud.pcd", *sortedPoints, false);*/

#if ENABLE_DISPLAY
		pcl::visualization::PCLVisualizer viewerS("SortedViewer");
		viewerS.addPointCloud(sortedPoints);
		viewerS.resetCamera();

		// 等待直到视图关闭
		while (!viewerS.wasStopped())
		{
			viewerS.spinOnce(100);
		}
#endif

		auto endOp = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsedOp = endOp - startOp;
		std::cout << "区域生长点排序算法用时: " << elapsedOp.count() << " seconds" << std::endl;

		// 创建可视化器
		pcl::visualization::PCLVisualizer viewer("Line Viewer");

		// 遍历点云并顺序连接相邻的点
		double dSamplingStep = 0.5;
		for (size_t i = 0; i < sortedPoints->points.size() - 1; ++i)
		{
			std::string line_id = "line_" + std::to_string(i);
			double dis = pcl::euclideanDistance(sortedPoints->points[i], sortedPoints->points[i + 1]);
			if (dis > dSamplingStep * mean)
				continue;

			viewer.addLine(sortedPoints->points[i], sortedPoints->points[i + 1], line_id);
		}
		// 连接最后一个点到第一个点，形成闭环
		double dis = pcl::euclideanDistance(sortedPoints->points.back(), sortedPoints->points.front());
		if (dis <= dSamplingStep * mean)
			viewer.addLine(sortedPoints->points.back(), sortedPoints->points.front(), "line_close");

		viewer.resetCamera();

		// 运行可视化器
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
		}

		vCloud.emplace_back(cloudCluster);
	}
}

void CurveReconstruct2(const PointCloud::Ptr& cloud)
{
	// 凹包排序
	pcl::ConcaveHull<PointT> concaveHull;
	concaveHull.setInputCloud(cloud);
	concaveHull.setAlpha(3.5);

	PointCloud::Ptr sortedPoints(new PointCloud);
	concaveHull.reconstruct(*sortedPoints);

	// 使用 PCL 可视化工具显示点云
	pcl::visualization::PCLVisualizer viewer1("Hull Viewer");
	viewer1.addPointCloud(sortedPoints);
	viewer1.resetCamera();

	// 等待直到视图关闭
	while (!viewer1.wasStopped()) { viewer1.spinOnce(); }

	// 计算均值和标准差
	const auto& [mean, stddev] = CalAvgStd(sortedPoints);

	// 欧几里德聚类提取器
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(sortedPoints);

	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(mean + stddev);							// 设置近邻搜索的搜索半径
	ec.setMinClusterSize(3);										// 设置一个聚类需要的最少点数目
	ec.setMaxClusterSize(100000);									// 设置一个聚类需要的最大点数目
	ec.setSearchMethod(tree);
	ec.setInputCloud(sortedPoints);
	ec.extract(clusterIndices);

	std::vector<PointCloud::Ptr> vCloud;
	for (const auto& indices : clusterIndices)
	{
		PointCloud::Ptr cloudCluster(new PointCloud);
		for (const auto& index : indices.indices)
			cloudCluster->points.push_back(sortedPoints->points[index]);

		// 创建可视化器
		pcl::visualization::PCLVisualizer viewer("Line Viewer");

		// 遍历点云并顺序连接相邻的点
		double dSamplingStep = 0.5;
		for (size_t i = 0; i < cloudCluster->points.size() - 1; ++i)
		{
			std::string line_id = "line_" + std::to_string(i);
			double dis = pcl::euclideanDistance(cloudCluster->points[i], cloudCluster->points[i + 1]);
			if (dis > dSamplingStep * stddev)
				continue;

			viewer.addLine(cloudCluster->points[i], cloudCluster->points[i + 1], line_id);
		}
		// 连接最后一个点到第一个点，形成闭环
		//viewer.addLine(sortedPoints->points.back(), sortedPoints->points.front(), "line_close");
		viewer.resetCamera();

		// 运行可视化器
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
		}

		vCloud.emplace_back(cloudCluster);
	}
}

int main(int argc, char** argv)
{
	PointCloud::Ptr cloud(new PointCloud);

	if (pcl::io::loadPCDFile<PointT>("pmt.pcd", *cloud) == -1)		// sac_plane_test.pcd | Scan_0511_1713.pcd
	{
		PCL_ERROR("点云读取失败 \n");
		return (-1);
	}
	cout << "Points num = " << cloud->points.size() << std::endl;
	auto startOp = std::chrono::high_resolution_clock::now();

	// 投影法
	//ProjMethod(cloud);

	// 求交法 IntersectMethodZ(cloud);
	PointCloud::Ptr sliceCloud(new PointCloud);
	IntersectMethod(sliceCloud, cloud);
	auto endOp = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp = endOp - startOp;
	std::cout << "Time: " << elapsedOp.count() << " seconds" << std::endl;

	// 多义线连接
	CurveReconstruct1(sliceCloud);

	return 0;
}