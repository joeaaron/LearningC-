#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/pointcloud_normal.h>
#include <vcg/complex/algorithms/clustering.h>

#include <wrap/io_trimesh/import_ply.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/pcd_io.h>

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

// �� PCL PolygonMesh ת��Ϊ VCG Mesh
void PCL2VCG(const pcl::PolygonMesh& pcl_mesh, MyMesh& vcg_mesh)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromPCLPointCloud2(pcl_mesh.cloud, cloud);

	// ��Ӷ��㵽 VCG Mesh
	vcg_mesh.vert.resize(cloud.points.size());
	for (size_t i = 0; i < cloud.points.size(); ++i) 
	{
		vcg_mesh.vert[i].P() = vcg::Point3f(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
	}

	vcg_mesh.vn = vcg_mesh.vert.size();  // ���¶�������

	// ����浽 VCG Mesh
	vcg_mesh.face.resize(pcl_mesh.polygons.size());
	for (size_t i = 0; i < pcl_mesh.polygons.size(); ++i)
	{
		const auto& polygon = pcl_mesh.polygons[i];
		vcg_mesh.face[i].V(0) = &vcg_mesh.vert[polygon.vertices[0]];
		vcg_mesh.face[i].V(1) = &vcg_mesh.vert[polygon.vertices[1]];
		vcg_mesh.face[i].V(2) = &vcg_mesh.vert[polygon.vertices[2]];
	}

	vcg_mesh.fn = vcg_mesh.face.size();  // ����������

	vcg::tri::UpdateBounding<MyMesh>::Box(vcg_mesh);						 // ���±߽��
	vcg::tri::UpdateNormal<MyMesh>::PerVertexNormalizedPerFace(vcg_mesh);    // ���·���
}

// ʹ�� VCG ������
void SimplifyVCGMesh(MyMesh& vcg_mesh, int targetFaceNum) 
{
	//vcg::LocalOptimization<MyMesh> Decimator(vcg_mesh);
	//vcg::tri::LocalOptimization<MyMesh>::Param Param;
	//Param.TargetFaces = targetFaceNum;
	//Decimator.Init(Param);
	//Decimator.DoOptimization();
}

int main(int argc, char** argv) 
{
	int target_face_num = 5000;
	pcl::PolygonMesh mesh;

	// ת�� PCL ����Ϊ VCG ����
	MyMesh vcg_mesh;
	PCL2VCG(mesh, vcg_mesh);

	// �� VCG ����
	SimplifyVCGMesh(vcg_mesh, target_face_num);

	//// ����򻯺�� VCG ����
	//if (!vcg::tri::io::Exporter<MyMesh>::Save(vcg_mesh, output_file.c_str())) {
	//	std::cerr << "Failed to save VCG mesh to " << output_file << std::endl;
	//	return -1;
	//}

	// ���岴���ؽ��Ĳ���
	//tri::Poisson<MyMesh>::Param poissonParam;
	//poissonParam.SetDensity(1.0);
	//poissonParam.SetScale(1.25);

	//// ִ�в����ؽ�
	//MyMesh mesh;
	//tri::Poisson<MyMesh>::Reconstruct(m, mesh, poissonParam);

	//// ����ϸ�����Ż�
	//// �������߳������Ƕ�
	//float maxEdgeLength = 0.05; // ���߳�
	//float maxAngle = 20.0; // ���Ƕȣ���λΪ����

	//// ���������Ĳ���
	//tri::TriEdgeCollapseQuadricParameter simplificationParam;
	//simplificationParam.QualityThr = 0.3; // ������ֵ
	//simplificationParam.PreserveBoundary = true; // ���ֱ߽�
	//simplificationParam.MaxEdgeLength = maxEdgeLength; // ���߳�
	//simplificationParam.MaxAngle = math::ToRad(maxAngle); // ���Ƕȣ�ת��Ϊ����

	//// ���ü�׼��
	//tri::TriEdgeCollapseQuadric::VertexPair key;
	//tri::TriEdgeCollapseQuadric::Heap::HeapElem elem;

	//// ������
	//tri::Simplification<MyMesh>::QuadricSimplification(mesh, simplificationParam);

	//// ������
	//if (tri::io::Exporter<MyMesh>::Save(mesh, "simplified_mesh.ply") != 0) {
	//	std::cerr << "Error saving file" << std::endl;
	//	return -1;
	//}

	return 0;
}