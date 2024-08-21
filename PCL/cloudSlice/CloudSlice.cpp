#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <omp.h> // ����OpenMP֧��
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/gp3.h>

#define ENABLE_DISPLAY 1		// ����һ���꣬���ڿ�����ʾ״̬
using PointT = pcl::PointXYZ;

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

	// ����OpenMP���л�
#pragma omp parallel for 
	// ����ԭʼ�����е�ÿ����  
	for (int i = 0; i < cloudIn->points.size(); ++i)
	{
		// ����һ��4x1���������������x, y, z, 1��  
		Eigen::Vector4d point_homogeneous(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z, 1.0);

		// Ӧ�ñ任���������������  
		Eigen::Vector4d transformed_point_homogeneous = transformT * point_homogeneous;

		// ��ȡ�任���3D���꣨x, y, z��  
		cloudout->points[i].x = transformed_point_homogeneous(0);
		cloudout->points[i].y = transformed_point_homogeneous(1);
		cloudout->points[i].z = transformed_point_homogeneous(2);
	}

	return cloudout;
}

inline Eigen::Vector4d CalcPlane(Eigen::Vector3d center, Eigen::Vector3d normal)
{
	Eigen::Vector4d planeEquation;

	// ����ƽ�淽���е� d
	double d = normal.dot(center);
	planeEquation << normal, -d;

	return planeEquation;
}

void GetSlice(pcl::PointCloud<pcl::PointXYZ>::Ptr& sliceCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Eigen::Vector4d& n)
{
	double delta = 2;			// ������Ƭ��0.5�����,��Ⱥ͵����ܶ����
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
	// ������
	//pcl::PCDWriter writer;
	//writer.write("sliceCloud.pcd", *sliceCloud, false);
	// ****************************��Χ���ڵ�����ʾ******************************
#if ENABLE_DISPLAY
	pcl::visualization::PCLVisualizer viewer("CropBox Viewer");
	viewer.addPointCloud(sliceCloud);
	viewer.resetCamera();
	// �ȴ�ֱ����ͼ�ر�
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
#endif
}

void ProjMethod(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	// ����ƽ��
	Eigen::Vector3d center(4.734, 9.470, 1.706);
	Eigen::Vector3d n(0.267, 0.535, 0.802);
	Eigen::Vector3d zDir(0, 0, 1);

	Eigen::Vector3d axis = n.cross(zDir);
	double angle = acos(n.dot(zDir) / n.norm());

	Eigen::AngleAxisd rotation(angle, axis.normalized());
	Eigen::Matrix3d rotationMatrix = rotation.toRotationMatrix();

	// ʹ����ת������ת����
	Eigen::Matrix4d transMtx = Eigen::Matrix4d::Identity();
	transMtx.block<3, 3>(0, 0) = rotationMatrix;

	pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	rotatedCloud = TransformCloud(cloud, transMtx);
	Eigen::Vector3d rotatedPt = rotationMatrix * center;

	// �õ���Χ���ڵ���
	Eigen::Vector4d plane = CalcPlane(rotatedPt, zDir);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sliceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	GetSlice(sliceCloud, rotatedCloud, plane);

	// ��Ƭ��ͶӰ��ƽ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	double d = -(zDir[0] * rotatedPt(0) + zDir[1] * rotatedPt(1) + zDir[2] * rotatedPt(2));
	for (size_t i = 0; i < sliceCloud->points.size(); ++i) {
		// ��ȡ��ǰ�������
		float x = sliceCloud->points[i].x;
		float y = sliceCloud->points[i].y;
		float z = sliceCloud->points[i].z;

		// ����㵽ƽ��Ĵ�ֱ����
		float distance = abs(zDir[0] * x + zDir[1] * y + zDir[2] * z - d) / sqrt(zDir[0] * zDir[0] + zDir[1] * zDir[1] + zDir[2] * zDir[2]);

		// ����ͶӰ�������
		float xp = x - distance * zDir[0] / sqrt(n[0] * zDir[0] + zDir[1] * zDir[1] + zDir[2] * zDir[2]);
		float yp = y - distance * zDir[1] / sqrt(n[0] * zDir[0] + zDir[1] * zDir[1] + zDir[2] * zDir[2]);
		float zp = z - distance * zDir[2] / sqrt(n[0] * zDir[0] + zDir[1] * zDir[1] + zDir[2] * zDir[2]);

		// ��ͶӰ������µĵ�����
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
	//-----------------�����ʾ---------------------
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
	// ***********************��ά͹���㷨*************************

	pcl::ConvexHull<PointT> convexHull;
	convexHull.setInputCloud(cloud);
	convexHull.setDimension(3);
	convexHull.setComputeAreaVolume(true);
	//����͹���е���Ҫ��
	std::vector<pcl::Vertices> polygons;
	pcl::PointCloud<PointT>::Ptr surface_hull(new pcl::PointCloud<PointT>);
	convexHull.reconstruct(*surface_hull, polygons);

	double v = convexHull.getTotalVolume();

	// ****************************���ӻ�******************************
	auto viewportsVis = [](pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudS,
		pcl::PointCloud<PointT>::Ptr surface_hull,
		std::vector<pcl::Vertices> polygons)
		{
			// ����3D���ڲ������ʾ�������������
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

// Step 1: ��������ܶ�
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

	//// ����ܶ���Ϣ������������
	//for (auto density : densities)
	//	std::cout << "Density: " << density << std::endl;
}

// Step 2: �õ����ҵ��� (������������Z����Ƭ)
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

// Step 3: ƥ�� lpoint �� rpoint �����㽻��
void FindCorres(pcl::PointCloud<pcl::PointXYZ>::Ptr lpoints, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr rpoints, double dPos, double threshold)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(rpoints);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sliceCloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (const auto& lpoint : lpoints->points) 
	{
		// ƽ���ϵĵ�ֱ�Ӽӽ�ȥ
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

				//// ���㽻�㣬����������Z������Ƭ
				pcl::PointXYZ intersection;
				//float ratio = (lpoint.z) / (lpoint.z - rpoint.z);
				//intersection.x = lpoint.x + ratio * (rpoint.x - lpoint.x);
				//intersection.y = lpoint.y + ratio * (rpoint.y - lpoint.y);
				//intersection.z = 0.0f;  // ��������Ƭƽ����

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

	// �ȴ�ֱ����ͼ�ر�
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
#endif

	std::cout << "Intersection points: " << sliceCloud->points.size() << std::endl;
}

void IntersectMethod(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	// Step 1: ��������ܶ�
	double dDelta = ComputeDensity(cloud, 100, 5);
	double dPos = -17.7557755;

	// Step 2: �õ����ҵ��� (������������Z����Ƭ)
	pcl::PointCloud<pcl::PointXYZ>::Ptr lpoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr rpoints(new pcl::PointCloud<pcl::PointXYZ>);
	SlicePointCloud(lpoints, rpoints, cloud, dPos, 10 * dDelta);

#if ENABLE_DISPLAY
	pcl::visualization::PCLVisualizer viewer("Viewer");
	viewer.addPointCloud(rpoints);
	viewer.resetCamera();

	// �ȴ�ֱ����ͼ�ر�
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
#endif

	// Step 3: ƥ�� lpoint �� rpoint �����㽻��
	FindCorres(lpoints, rpoints, dPos, 8 * dDelta);
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("pmt.pcd", *cloud) == -1)		// sac_plane_test.pcd | Scan_0511_1713.pcd
	{
		PCL_ERROR("���ƶ�ȡʧ�� \n");
		return (-1);
	}

	// ͶӰ��
	//ProjMethod(cloud);

	// �󽻷�
	IntersectMethod(cloud);

	return 0;
}