#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkTriangle.h>
#include <pcl/common/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#define ENABLE_DISPLAY 1				// 定义一个宏，用于控制显示状态

const int SAMPLE_POINTS = 100000;
bool bCalNormal = true;
bool bCalColor = false;

bool Mesh2CloudPCL(pcl::PointCloud<pcl::PointXYZ>& cloudOut,
	const pcl::PolygonMesh& mesh)
{
	pcl::fromPCLPointCloud2(mesh.cloud, cloudOut);
	return true;
}

double UniformDeviate(int seed)
{
	double ran = seed * (1.0 / (RAND_MAX + 1.0));
	return ran;
}

/**
 * @brief 在三角形内随机生成一个点
 *
 * 通过给定的三角形三个顶点的坐标（a, b, c）和两个随机数（r1, r2），计算出一个在三角形内部的随机点。
 *
 * @param a1, a2, a3 第一个顶点的三维坐标
 * @param b1, b2, b3 第二个顶点的三维坐标
 * @param c1, c2, c3 第三个顶点的三维坐标
 * @param r1, r2 两个0到1之间的随机数
 * @param[out] p 计算出的随机点的三维坐标，以Eigen::Vector3d类型返回
 */
void RandomPointTriangle(float a1, float a2, float a3, 
	float b1, float b2, float b3, 
	float c1, float c2, float c3,
	float r1, float r2, 
	Eigen::Vector3d& p)
{
	float r1sqr = std::sqrt(r1);
	float oneMinR1Sqr = (1 - r1sqr);
	float oneMinR2 = (1 - r2);

	a1 *= oneMinR1Sqr;
	a2 *= oneMinR1Sqr;
	a3 *= oneMinR1Sqr;

	b1 *= oneMinR2;
	b2 *= oneMinR2;
	b3 *= oneMinR2;

	c1 = r1sqr * (r2 * c1 + b1) + a1;
	c2 = r1sqr * (r2 * c2 + b2) + a2;
	c3 = r1sqr * (r2 * c3 + b3) + a3;

	p[0] = c1;
	p[1] = c2;
	p[2] = c3;
}

void RandPSurface(vtkPolyData* polydata, std::vector<double>* cumulativeAreas, double totalArea, 
	Eigen::Vector3d& p,
	Eigen::Vector3d& n, 
	Eigen::Vector3d& c)
{
	float r = static_cast<float> (UniformDeviate(rand()) * totalArea);

	std::vector<double>::iterator low = std::lower_bound(cumulativeAreas->begin(), cumulativeAreas->end(), r);
	vtkIdType el = vtkIdType(low - cumulativeAreas->begin());

	double A[3], B[3], C[3];
	vtkIdType npts = 0;
	vtkIdType* ptIds = nullptr;
	vtkNew<vtkIdList> idList;

	polydata->GetCellPoints(el, idList);
	polydata->GetPoint(idList->GetId(0), A);
	polydata->GetPoint(idList->GetId(1), B);
	polydata->GetPoint(idList->GetId(2), C);

	if (bCalNormal)
	{
		// OBJ: Vertices are stored in a counter-clockwise order by default
		Eigen::Vector3d v1 = Eigen::Vector3d(A[0], A[1], A[2]) - Eigen::Vector3d(C[0], C[1], C[2]);
		Eigen::Vector3d v2 = Eigen::Vector3d(B[0], B[1], B[2]) - Eigen::Vector3d(C[0], C[1], C[2]);
		n = v1.cross(v2);
		n.normalize();
	}

	float r1 = static_cast<float> (UniformDeviate(rand()));
	float r2 = static_cast<float> (UniformDeviate(rand()));

	RandomPointTriangle(float(A[0]), float(A[1]), float(A[2]),
		float(B[0]), float(B[1]), float(B[2]),
		float(C[0]), float(C[1]), float(C[2]), 
		r1, r2, p);

	if (bCalColor)
	{
		vtkUnsignedCharArray* const colors = vtkUnsignedCharArray::SafeDownCast(polydata->GetPointData()->GetScalars());
		if (colors && colors->GetNumberOfComponents() == 3)
		{
			double cA[3], cB[3], cC[3];
			colors->GetTuple(ptIds[0], cA);
			colors->GetTuple(ptIds[1], cB);
			colors->GetTuple(ptIds[2], cC);

			RandomPointTriangle(float(cA[0]), float(cA[1]), float(cA[2]),
				float(cB[0]), float(cB[1]), float(cB[2]),
				float(cC[0]), float(cC[1]), float(cC[2]), r1, r2, c);
		}
		else
		{
			static bool printed_once = false;
			if (!printed_once)
				PCL_WARN("Mesh has no vertex colors, or vertex colors are not RGB!");
			printed_once = true;
		}
	}
}

bool Mesh2Cloud(pcl::PointCloud<pcl::PointXYZ>& cloudOut,
	const pcl::PolygonMesh& mesh)
{
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::mesh2vtk(mesh, polyData);

	if (polyData == nullptr) return false;

	polyData->BuildCells();

	double p1[3], p2[3], p3[3], totalArea = 0;
	std::size_t cellId = 0;
	vtkNew<vtkIdList> idList;
	vtkSmartPointer<vtkCellArray> cells = polyData->GetPolys();
	std::vector<double> cumulativeAreas(cells->GetNumberOfCells(), 0);

	if (cells == nullptr) return false;
	for (cells->InitTraversal(); cells->GetNextCell(idList); cellId++)
	{
		polyData->GetPoint(idList->GetId(0), p1);
		polyData->GetPoint(idList->GetId(1), p2);
		polyData->GetPoint(idList->GetId(2), p3);

		totalArea += vtkTriangle::TriangleArea(p1, p2, p3);
		cumulativeAreas[cellId] = totalArea;
	}

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudAll(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	cloudAll->points.resize(SAMPLE_POINTS);
	cloudAll->width = static_cast<std::uint32_t> (SAMPLE_POINTS);
	cloudAll->height = 1;

	for (size_t i = 0; i < SAMPLE_POINTS; ++i)
	{
		Eigen::Vector3d pt;
		Eigen::Vector3d normal(0, 0, 0);
		Eigen::Vector3d color(0, 0, 0);

		RandPSurface(polyData, &cumulativeAreas, totalArea, pt, normal, color);

		cloudAll->points[i].x = pt[0];
		cloudAll->points[i].y = pt[1];
		cloudAll->points[i].z = pt[2];

		if (bCalNormal)
		{
			cloudAll->points[i].normal_x = normal[0];
			cloudAll->points[i].normal_y = normal[1];
			cloudAll->points[i].normal_z = normal[2];
		}
		if (bCalColor)
		{
			cloudAll->points[i].r = static_cast<std::uint8_t>(color[0]);
			cloudAll->points[i].g = static_cast<std::uint8_t>(color[1]);
			cloudAll->points[i].b = static_cast<std::uint8_t>(color[2]);
		}
	}

	// Voxelgrid
	pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
	grid.setInputCloud(cloudAll);
	grid.setLeafSize(0.01, 0.01, 0.01);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	grid.filter(*voxel_cloud);

	pcl::copyPointCloud(*voxel_cloud, cloudOut);

	return true;
}

int main()
{
	pcl::PolygonMesh mesh;
	if(pcl::io::loadPolygonFileSTL("test.stl", mesh) == -1)
	{
		PCL_ERROR("STL读取失败 \n");
		return (-1);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	Mesh2CloudPCL(*cloud1, mesh);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	Mesh2Cloud(*cloud2, mesh);

#if ENABLE_DISPLAY
	//-----------------结果显示---------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Mesh to cloud"));

	// 合在一起显示
	/*int v1(0), v2(0);
	viewer->setWindowName(u8"网格化后的点云");
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	viewer->addPointCloud(cloudXYZ, "cloudRdm");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloudRdm");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloudRdm");*/

	// 分2个窗口进行显示
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v1);
	viewer->addText("cloud", 10, 10, "v1_text", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
	viewer->setBackgroundColor(0.5, 0.5, 0.5, v2);
	viewer->addText("cloudUnm", 10, 10, "v2_text", v2);

	viewer->addPointCloud<pcl::PointXYZ>(cloud1, "cloud", v1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud2, "cloudUnm", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloudUnm", v2);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(1000);
	}
#endif
	return 0;
}