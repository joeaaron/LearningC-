#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h> 

#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/ear_clipping.h> 
#include <pcl/surface/vtk_smoothing/vtk_utils.h> 
#include <pcl/filters/uniform_sampling.h>
#include <vtkDecimatePro.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>

#define ENABLE_DISPLAY 1			  // ����һ���꣬���ڿ�����ʾ״̬

void PreprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// ͳ���˲�
	/*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(10);
	sor.setStddevMulThresh(0.5);
	sor.filter(*cloud);*/

	// ���Ȳ���
	//pcl::UniformSampling<pcl::PointXYZ> uniform;
	//uniform.setInputCloud(cloud);
	//uniform.setRadiusSearch(0.25);
	//uniform.filter(*cloud);

	// ���ز���
	//pcl::VoxelGrid<pcl::PointXYZ> vg;
	//vg.setInputCloud(cloud);
	//vg.setLeafSize(0.25, 0.25, 0.25);
	//vg.filter(*cloud);
}

void GreedyTriangle(pcl::PolygonMesh& triangles, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals)
{
	pcl::console::TicToc time;
	time.tic();

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//��������������
	tree2->setInputCloud(cloud_with_normals);				//����������ƹ���tree

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//�������ǻ�����

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(10);								//���������뾶radius����ȷ�����ǻ�ʱkһ�ڽ�����뾶��

	// Set typical values for the parameters
	gp3.setMu(3);							 //���������㵽����������ĳ˻�ϵ�� mu �����ÿ�������������������룬����ʹ���㷨����Ӧ�����ܶȵı仯
	gp3.setMaximumNearestNeighbors(100);	 //����������������������������Ŀ100 ��
	gp3.setMaximumSurfaceAngle(M_PI / 4);    //45 degrees����������ʱ�����Ƕ� eps_angle ����ĳ�㷨������ڲ�����ķ���ƫ��Ƕȳ��������Ƕ�ʱ������ʱ�Ͳ����Ǹõ㡣
	gp3.setMinimumAngle(M_PI / 5);           //10 degrees���������ǻ��������ε���С�ǣ����� minimum_angle Ϊ��С�ǵ�ֵ��
	gp3.setMaximumAngle(2 * M_PI / 3);		 //120 degrees���������ǻ��������ε����ǣ����� maximum_angle Ϊ���ǵ�ֵ��
	gp3.setNormalConsistency(false);		 //����һ����־ consistent ������֤���߳���һ�£��������Ϊ true ���ʹ���㷨���ַ��߷���һ�£����Ϊ false �㷨�򲻻���з���һ���Լ�顣
	// Get result
	gp3.setInputCloud(cloud_with_normals);	 //�����������Ϊ�������
	gp3.setSearchMethod(tree2);				 //����������ʽtree2
	gp3.reconstruct(triangles);				 //�ؽ���ȡ���ǻ�

	// std::cout << triangles;
	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();//����ؽ���ÿ��� ID, Parts �� 0 ��ʼ��ţ� a-1 ��ʾδ���ӵĵ㡣
	/*
	����ؽ���ÿ���״̬��ȡֵΪ FREE �� FRINGE �� BOUNDARY �� COMPLETED �� NONE ������
	���� NONE ��ʾδ���壬
	FREE ��ʾ�õ�û�������ǻ���������ڣ�Ϊ���ɵ㣬
	COMPLETED ��ʾ�õ������ǻ���������ڣ��������������˵㣬
	BOUNDARY ��ʾ�õ������ǻ�������˱�Ե��
	FRINGE ��ʾ�õ������ǻ���������ڣ������ӻ�����ص��ߡ�
	*/
	std::vector<int> states = gp3.getPointStates();

	cout << "�������ǻ��㷨��ʱ�� " << time.toc() / 1000 << " ��" << endl;

#if ENABLE_DISPLAY
	//----------------------------------������ӻ�-----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"̰��ͶӰ���ǻ�");
	//int v1(0), v2(0);
	//viewer->createViewPort(0, 0, 0.5, 1, v1);
	////viewer->createViewPort(0.5, 0, 1, 1, v2);
	//viewer->setBackgroundColor(0, 0, 0, v1);
	//viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");	 // ���ӻ�����
	viewer->addPolygonMesh(triangles, "my");                 // ���ӻ�ģ���ؽ����
	viewer->setRepresentationToSurfaceForAllActors();            // ����ģ�����߿�ͼģʽ��ʾ
	//viewer->addCoordinateSystem(0.2);
	viewer->initCameraParameters();
	viewer->spin();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
}

void PossionTriangle(pcl::PolygonMesh& triangles, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals)
{
	pcl::console::TicToc time;
	time.tic();

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//��������������
	tree2->setInputCloud(cloud_with_normals);				//����������ƹ���tree

	pcl::Poisson<pcl::PointNormal> pn;
	pn.setSearchMethod(tree2);
	pn.setInputCloud(cloud_with_normals);
	pn.setThreads(12);										// ���̼߳���
	pn.setDepth(6);											// ���ý����ڱ����ؽ�������������
	pn.setMinDepth(2);
	pn.setScale(1.25);										// ���������ؽ����������ֱ���������ı߽�������ֱ���ı�ֵ
	pn.setSolverDivide(3);									// ���ÿ��˹-���¶�������������������˹���̵���ȡ�
	pn.setIsoDivide(6);										// ���ÿ�ȱ�����ȡ��������ȡ�ȱ�������
	pn.setSamplesPerNode(10);								// ����ÿ���˲����ڵ������ٲ�������Ŀ
	pn.setConfidence(false);								// �������ű�־��Ϊtrueʱ��ʹ�÷�������������Ϊ���Ŷ���Ϣ��false����Ҫ�Է��߽��й�һ������
	pn.setManifold(false);									// �������б�־���������Ϊtrue����Զ���ν���ϸ�����ǻ�ʱ������ģ�����false�����
	pn.setOutputPolygons(false);							// �����Ƿ����Ϊ�����(���������ǻ��н�������Ľ��)��
	pn.performReconstruction(triangles);

	cout << "���������ؽ��㷨��ʱ�� " << time.toc() / 1000 << " ��" << endl;

#if ENABLE_DISPLAY
	//----------------------------------������ӻ�-----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"���������ؽ�");
	int v1(0), v2(0);
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);	 // ���ӻ�����
	viewer->addPolygonMesh(triangles, "my", v2);                 // ���ӻ�ģ���ؽ����
	viewer->setRepresentationToSurfaceForAllActors();            // ����ģ�����߿�ͼģʽ��ʾ
	//viewer->addCoordinateSystem(0.2);
	viewer->initCameraParameters();
	viewer->spin();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
}

void GridTriangle(pcl::PolygonMesh& triangles,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals)
{
	pcl::console::TicToc time;
	time.tic();

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//��������������
	tree2->setInputCloud(cloud_with_normals);				//����������ƹ���tree

	pcl::GridProjection<pcl::PointNormal> gp;
	gp.setInputCloud(cloud_with_normals);
	gp.setSearchMethod(tree2);
	gp.setResolution(0.005);			// ����ֱ���
	gp.setPaddingSize(3);				// ��������䵥Ԫ�������
	gp.reconstruct(triangles);			// �����ؽ�

	cout << "����ͶӰ�����ؽ��㷨��ʱ�� " << time.toc() / 1000 << " ��" << endl;

#if ENABLE_DISPLAY
	//----------------------------------������ӻ�-----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"����ͶӰ�����ؽ�");
	int v1(0), v2(0);
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);	 // ���ӻ�����
	viewer->addPolygonMesh(triangles, "my", v2);                 // ���ӻ�ģ���ؽ����
	viewer->setRepresentationToSurfaceForAllActors();            // ����ģ�����߿�ͼģʽ��ʾ
	//viewer->addCoordinateSystem(0.2);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
}

void MarchingCubesTriangles(pcl::PolygonMesh& triangles,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals)
{
	pcl::console::TicToc time;
	time.tic();

	//---------��ʼ��MarchingCubes���󣬲����ò���-------
	pcl::MarchingCubes<pcl::PointNormal>* mc;
	mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
	//����������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	mc->setInputCloud(cloud_with_normals);
	//����MarchingCubes����Ĳ���
	mc->setIsoLevel(0.0f);//�÷�������Ҫ��ȡ�����iso����
	mc->setGridResolution(50, 50, 50);//���������н�����������ֱ���
	mc->setPercentageExtendGrid(0.0f);//�ò��������ڵ��Ƶı߿����������֮���������Ӧ�ñ����������ɿռ�
	//------����������������ڴ洢���------------
	mc->reconstruct(triangles);//ִ���ع������������mesh��

	cout << "�ƶ��������㷨��ʱ�� " << time.toc() / 1000 << " ��" << endl;

#if ENABLE_DISPLAY
	//----------------------------------������ӻ�-----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"�ƶ�������");
	int v1(0), v2(0);
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);	 // ���ӻ�����
	viewer->addPolygonMesh(triangles, "my", v2);                 // ���ӻ�ģ���ؽ����
	viewer->setRepresentationToSurfaceForAllActors();            // ����ģ�����߿�ͼģʽ��ʾ
	//viewer->addCoordinateSystem(0.2);
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
}

void EarClippintTriangles(pcl::PolygonMesh& triangles, const pcl::PolygonMesh::Ptr& mesh)
{
	pcl::console::TicToc time;
	time.tic();

	// --------------------------����ü������ʷ��㷨---------------------------
	pcl::EarClipping clipper;
	pcl::PolygonMesh::ConstPtr mesh_aux(mesh);
	clipper.setInputMesh(mesh_aux);

	clipper.process(triangles);

	cout << "����ü������ʷ��㷨��ʱ�� " << time.toc() / 1000 << " ��" << endl;

#if ENABLE_DISPLAY
	//-------------------------------������ӻ�----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	int v1(0), v2(0);
	viewer->setWindowName(u8"�����㷨");
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);

	viewer->addPolygonMesh(*mesh, "my1", v1);
	viewer->addPolygonMesh(triangles, "my", v2);
	viewer->setRepresentationToSurfaceForAllActors();			//����ģ�����߿�ͼģʽ��ʾ

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
}

int
main(int argc, char** argv)
{
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud) == -1)		// sac_plane_test.pcd | 800w.pcd | table_scene_lms400.pcd
	{
		PCL_ERROR("���ƶ�ȡʧ�� \n");
		return (-1);
	} //* the data should be available in cloud

	// ����Ԥ����
	PreprocessPointCloud(cloud);

	// Normal estimation
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;								//���÷��߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);			//�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);	//����kd��ָ��
	tree->setInputCloud(cloud);																//��cloud����tree����
	ne.setInputCloud(cloud);																//Ϊ���߹��ƶ��������������
	ne.setSearchMethod(tree);																//������������
	ne.setKSearch(100);																		//����k�������ص�������Χ
	ne.setNumberOfThreads(4);
	ne.compute(*normals);																	//���Ʒ���
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);//
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);							//�����ֶΣ�cloud_with_normals�洢�������
	//* cloud_with_normals = cloud + normals

	pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
	//GreedyTriangle(*triangles, cloud, cloud_with_normals);					// ̰��ͶӰ����
	//PossionTriangle(triangles, cloud, cloud_with_normals);				// �����ؽ�
	//GridTriangle(triangles, cloud, cloud_with_normals);					// ����ͶӰ�����ؽ�
	MarchingCubesTriangles(*triangles, cloud, cloud_with_normals);			// �ƶ�������
	std::string output_filename = "output_mesh.stl";

	// ����Ϊ STL �ļ�
	if (pcl::io::savePolygonFileSTL(output_filename, *triangles) == 0)
	{
		std::cout << "Successfully saved mesh to " << output_filename << std::endl;
	}
	else
	{
		std::cerr << "Failed to save mesh to " << output_filename << std::endl;
		return -1;
	}
	// -----------------------------��ȡmesh����-------------------------------
	//pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
	//if (pcl::io::loadPolygonFileVTK("tum_rabbit.vtk", *mesh) == -1)			// tum_rabbit.vtk | sphere.vtk"
	//{
	//	std::cerr << "���ݶ�ȡʧ�ܣ�����" << std::endl;
	//}
	//EarClippintTriangles(triangles, mesh);									 // ���������ʷ�

	size_t polygonNums = triangles->polygons.size();
	std::cout << "��Ƭ����" << polygonNums << std::endl;

	//for (int i = 0; i < polygonNums; ++i)
	//{
	//	std::cout << "polygon" << i << "has" << triangles.polygons[i].vertices.size() << "vertices." << std::endl;
	//}
	
	// ʹ��Laplacian��˳�㷨
	//pcl::MeshSmoothingLaplacianVTK smoother;
	//pcl::PolygonMesh smoothed_mesh;
	//smoother.setInputMesh(triangles);
	//smoother.setNumIter(100); // ���õ�������
	//smoother.setConvergence(0.001); // ����������׼
	//smoother.setRelaxationFactor(0.1); // �����ɳ�����
	//smoother.process(smoothed_mesh);

	// ��˳���ı���Ƭ��
	//polygonNums = smoothed_mesh.polygons.size();
	//std::cout << "��˳����Ƭ����" << polygonNums << std::endl;

//#if ENABLE_DISPLAY
//	//----------------------------------������ӻ�-----------------------------------
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setWindowName(u8"��˳���ģ��");
//	//viewer->addPointCloud(cloud, "cloud");
//	viewer->addPolygonMesh(smoothed_mesh, "my");                 // ���ӻ�ģ���ؽ����
//	viewer->setRepresentationToSurfaceForAllActors();            // ����ģ�����߿�ͼģʽ��ʾ
//	//viewer->addCoordinateSystem(0.2);
//	viewer->initCameraParameters();
//	viewer->resetCamera();
//	viewer->spin();
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);
//		
//	}
//#endif

	//// �����
	//vtkSmartPointer<vtkPolyData> vtkMesh;
	//pcl::VTKUtils::convertToVTK(smoothed_mesh, vtkMesh);

	//// ʹ��VTK��DecimatePro ���м�
	//vtkSmartPointer<vtkDecimatePro> decimate = vtkSmartPointer<vtkDecimatePro>::New();
	//decimate->SetInputData(vtkMesh);
	//decimate->SetTargetReduction(0.5);   // ����50%����
	//decimate->Update();

	//// ���򻯺��VTK����ת��PCL
	//vtkSmartPointer<vtkPolyData> simplifiedVTKMesh = decimate->GetOutput();
	//pcl::PolygonMesh simplifiedMesh;
	//pcl::VTKUtils::convertToPCL(simplifiedVTKMesh, simplifiedMesh);

	//polygonNums = simplifiedMesh.polygons.size();
	//std::cout << "�򻯺����Ƭ����" << polygonNums << std::endl;

//#if ENABLE_DISPLAY
//	//----------------------------------������ӻ�-----------------------------------
//	viewer->setWindowName(u8"�򻯺����Ƭ");
//	int v1(0), v2(0);
//	viewer->createViewPort(0, 0, 0.5, 1, v1);
//	viewer->createViewPort(0.5, 0, 1, 1, v2);
//	viewer->setBackgroundColor(0, 0, 0, v1);
//	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
//	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);	  // ���ӻ�����
//	viewer->addPolygonMesh(simplifiedMesh, "my", v2);             // ���ӻ�ģ���ؽ����
//	viewer->setRepresentationToSurfaceForAllActors();             // ����ģ�����߿�ͼģʽ��ʾ
//	//viewer->initCameraParameters();
//	viewer->initCameraParameters();
//	viewer->spin();
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);
//	}
//#endif
//
//	return (0);
}