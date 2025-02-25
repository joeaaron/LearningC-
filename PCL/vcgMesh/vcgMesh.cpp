#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/pointcloud_normal.h>
#include <vcg/complex/algorithms/clustering.h>

#include <wrap/io_trimesh/import_ply.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/pcd_io.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkProperty.h>
using namespace vcg;
using namespace std;

class MyEdge;
class MyFace;
class MyVertex;
struct MyUsedTypes : public UsedTypes<	Use<MyVertex>   ::AsVertexType,
	Use<MyEdge>     ::AsEdgeType,
	Use<MyFace>     ::AsFaceType> {};

class MyVertex : public Vertex<MyUsedTypes, vertex::Coord3f, vertex::Normal3f, vertex::VFAdj, vertex::Qualityf, vertex::BitFlags, vertex::Mark> {};
class MyFace : public Face< MyUsedTypes, face::Mark, face::VertexRef, face::VFAdj, face::FFAdj, face::Normal3f, face::BitFlags > {};
class MyEdge : public Edge<MyUsedTypes> {};
class MyMesh : public tri::TriMesh< vector<MyVertex>, vector<MyFace>, vector<MyEdge>  > {};

// 将 PCL PolygonMesh 转换为 VCG Mesh
void PCL2VCG(const pcl::PolygonMesh& pcl_mesh, MyMesh& vcg_mesh)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromPCLPointCloud2(pcl_mesh.cloud, cloud);

	// 添加顶点到 VCG Mesh
	vcg_mesh.vert.resize(cloud.points.size());
	for (size_t i = 0; i < cloud.points.size(); ++i) 
	{
		vcg_mesh.vert[i].P() = vcg::Point3f(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
	}

	vcg_mesh.vn = vcg_mesh.vert.size();  // 更新顶点数量

	// 添加面到 VCG Mesh
	vcg_mesh.face.resize(pcl_mesh.polygons.size());
	for (size_t i = 0; i < pcl_mesh.polygons.size(); ++i)
	{
		const auto& polygon = pcl_mesh.polygons[i];
		vcg_mesh.face[i].V(0) = &vcg_mesh.vert[polygon.vertices[0]];
		vcg_mesh.face[i].V(1) = &vcg_mesh.vert[polygon.vertices[1]];
		vcg_mesh.face[i].V(2) = &vcg_mesh.vert[polygon.vertices[2]];
	}

	vcg_mesh.fn = vcg_mesh.face.size();  // 更新面数量

	vcg::tri::UpdateBounding<MyMesh>::Box(vcg_mesh);						 // 更新边界框
	vcg::tri::UpdateNormal<MyMesh>::PerVertexNormalizedPerFace(vcg_mesh);    // 更新法线
}

// 使用 VCG 简化网格
void SimplifyVCGMesh(MyMesh& vcg_mesh, int targetFaceNum) 
{
	//vcg::LocalOptimization<MyMesh> Decimator(vcg_mesh);
	//vcg::tri::LocalOptimization<MyMesh>::Param Param;
	//Param.TargetFaces = targetFaceNum;
	//Decimator.Init(Param);
	//Decimator.DoOptimization();
}

void UpdateDisplay()
{
	//if (vcgMesh == nullptr || vcgMesh->vn == 0 || vcgMesh->fn == 0) return;

	//auto pclMesh = ConvertVCGToPCL(vcgMesh);
	//if (!pclMesh) return;

	//// 1. 创建 VTK 点
	//auto vtkpoints = vtkSmartPointer<vtkPoints>::New();
	//pcl::PointCloud<pcl::PointXYZ> cloud;
	//pcl::fromPCLPointCloud2(pclMesh->cloud, cloud);  // 从 PCL 网格提取点云
	//for (const auto& point : cloud.points) {
	//	vtkpoints->InsertNextPoint(point.x, point.y, point.z);
	//}

	//// 2. 创建 VTK 面
	//auto vtkCells = vtkSmartPointer<vtkCellArray>::New();
	//for (const auto& polygon : pclMesh->polygons) {
	//	if (polygon.vertices.size() != 3) continue;  // 仅支持三角形面

	//	auto vtktriangle = vtkSmartPointer<vtkTriangle>::New();
	//	vtktriangle->GetPointIds()->SetId(0, polygon.vertices[0]);
	//	vtktriangle->GetPointIds()->SetId(1, polygon.vertices[1]);
	//	vtktriangle->GetPointIds()->SetId(2, polygon.vertices[2]);
	//	vtkCells->InsertNextCell(vtktriangle);
	//}

	//// 3. 创建 VTK PolyData
	//auto polyData = vtkSmartPointer<vtkPolyData>::New();
	//polyData->SetPoints(vtkpoints);
	//polyData->SetPolys(vtkCells);

	//// 4. 创建 Mapper 和 Actor
	//auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	//mapper->SetInputData(polyData);

	//auto actor = vtkSmartPointer<vtkActor>::New();
	//actor->SetMapper(mapper);
	//actor->GetProperty()->SetColor(0.8, 0.8, 0.8);  // 设置颜色

	//// 5. 渲染管线
	//auto renderer = vtkSmartPointer<vtkRenderer>::New();
	//renderer->AddActor(actor);
	//renderer->SetBackground(0.1, 0.1, 0.1);  // 设置背景色

	//auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	//renderWindow->AddRenderer(renderer);

	//auto renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	//renderWindowInteractor->SetRenderWindow(renderWindow);

	//// 6. 显示
	//renderWindow->Render();
	//renderWindowInteractor->Start();
	auto points = vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(0.0, 0.0, 0.0);
	points->InsertNextPoint(1.0, 0.0, 0.0);
	points->InsertNextPoint(0.0, 1.0, 0.0);

	auto triangle = vtkSmartPointer<vtkTriangle>::New();
	triangle->GetPointIds()->SetId(0, 0);
	triangle->GetPointIds()->SetId(1, 1);
	triangle->GetPointIds()->SetId(2, 2);

	auto cells = vtkSmartPointer<vtkCellArray>::New();
	cells->InsertNextCell(triangle);

	auto polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData->SetPoints(points);
	polyData->SetPolys(cells);

	auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polyData);

	auto actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	auto renderer = vtkSmartPointer<vtkRenderer>::New();
	renderer->AddActor(actor);
	renderer->SetBackground(0.1, 0.1, 0.1);

	auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);

	auto renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderWindow->Render();
	renderWindowInteractor->Start();
}

int main(int argc, char** argv) 
{
	int target_face_num = 5000;
	pcl::PolygonMesh mesh;

	// 转换 PCL 网格为 VCG 网格
	/*MyMesh vcg_mesh;
	PCL2VCG(mesh, vcg_mesh);*/
	UpdateDisplay();

	// 简化 VCG 网格
	//SimplifyVCGMesh(vcg_mesh, target_face_num);

	//// 保存简化后的 VCG 网格
	//if (!vcg::tri::io::Exporter<MyMesh>::Save(vcg_mesh, output_file.c_str())) {
	//	std::cerr << "Failed to save VCG mesh to " << output_file << std::endl;
	//	return -1;
	//}

	// 定义泊松重建的参数
	//tri::Poisson<MyMesh>::Param poissonParam;
	//poissonParam.SetDensity(1.0);
	//poissonParam.SetScale(1.25);

	//// 执行泊松重建
	//MyMesh mesh;
	//tri::Poisson<MyMesh>::Reconstruct(m, mesh, poissonParam);

	//// 网格细化和优化
	//// 定义最大边长和最大角度
	//float maxEdgeLength = 0.05; // 最大边长
	//float maxAngle = 20.0; // 最大角度，单位为度数

	//// 定义简化网格的参数
	//tri::TriEdgeCollapseQuadricParameter simplificationParam;
	//simplificationParam.QualityThr = 0.3; // 质量阈值
	//simplificationParam.PreserveBoundary = true; // 保持边界
	//simplificationParam.MaxEdgeLength = maxEdgeLength; // 最大边长
	//simplificationParam.MaxAngle = math::ToRad(maxAngle); // 最大角度，转换为弧度

	//// 设置简化准则
	//tri::TriEdgeCollapseQuadric::VertexPair key;
	//tri::TriEdgeCollapseQuadric::Heap::HeapElem elem;

	//// 简化网格
	//tri::Simplification<MyMesh>::QuadricSimplification(mesh, simplificationParam);

	//// 保存结果
	//if (tri::io::Exporter<MyMesh>::Save(mesh, "simplified_mesh.ply") != 0) {
	//	std::cerr << "Error saving file" << std::endl;
	//	return -1;
	//}

	return 0;
}