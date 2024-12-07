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

#define ENABLE_DISPLAY 1			  // 定义一个宏，用于控制显示状态

void PreprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// 统计滤波
	/*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(10);
	sor.setStddevMulThresh(0.5);
	sor.filter(*cloud);*/

	// 均匀采样
	//pcl::UniformSampling<pcl::PointXYZ> uniform;
	//uniform.setInputCloud(cloud);
	//uniform.setRadiusSearch(0.25);
	//uniform.filter(*cloud);

	// 体素采样
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
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//定义搜索树对象
	tree2->setInputCloud(cloud_with_normals);				//利用有向点云构造tree

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(10);								//设置搜索半径radius，来确定三角化时k一邻近的球半径。

	// Set typical values for the parameters
	gp3.setMu(3);							 //设置样本点到最近邻域距离的乘积系数 mu 来获得每个样本点的最大搜索距离，这样使得算法自适应点云密度的变化
	gp3.setMaximumNearestNeighbors(100);	 //设置样本点最多可以搜索的邻域数目100 。
	gp3.setMaximumSurfaceAngle(M_PI / 4);    //45 degrees，设置连接时的最大角度 eps_angle ，当某点法线相对于采样点的法线偏离角度超过该最大角度时，连接时就不考虑该点。
	gp3.setMinimumAngle(M_PI / 5);           //10 degrees，设置三角化后三角形的最小角，参数 minimum_angle 为最小角的值。
	gp3.setMaximumAngle(2 * M_PI / 3);		 //120 degrees，设置三角化后三角形的最大角，参数 maximum_angle 为最大角的值。
	gp3.setNormalConsistency(false);		 //设置一个标志 consistent ，来保证法线朝向一致，如果设置为 true 则会使得算法保持法线方向一致，如果为 false 算法则不会进行法线一致性检查。
	// Get result
	gp3.setInputCloud(cloud_with_normals);	 //设置输入点云为有向点云
	gp3.setSearchMethod(tree2);				 //设置搜索方式tree2
	gp3.reconstruct(triangles);				 //重建提取三角化

	// std::cout << triangles;
	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();//获得重建后每点的 ID, Parts 从 0 开始编号， a-1 表示未连接的点。
	/*
	获得重建后每点的状态，取值为 FREE 、 FRINGE 、 BOUNDARY 、 COMPLETED 、 NONE 常量，
	其中 NONE 表示未定义，
	FREE 表示该点没有在三角化后的拓扑内，为自由点，
	COMPLETED 表示该点在三角化后的拓扑内，并且邻域都有拓扑点，
	BOUNDARY 表示该点在三角化后的拓扑边缘，
	FRINGE 表示该点在三角化后的拓扑内，其连接会产生重叠边。
	*/
	std::vector<int> states = gp3.getPointStates();

	cout << "点云三角化算法用时： " << time.toc() / 1000 << " 秒" << endl;

#if ENABLE_DISPLAY
	//----------------------------------结果可视化-----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"贪婪投影三角化");
	//int v1(0), v2(0);
	//viewer->createViewPort(0, 0, 0.5, 1, v1);
	////viewer->createViewPort(0.5, 0, 1, 1, v2);
	//viewer->setBackgroundColor(0, 0, 0, v1);
	//viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");	 // 可视化点云
	viewer->addPolygonMesh(triangles, "my");                 // 可视化模型重建结果
	viewer->setRepresentationToSurfaceForAllActors();            // 网格模型以线框图模式显示
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
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//定义搜索树对象
	tree2->setInputCloud(cloud_with_normals);				//利用有向点云构造tree

	pcl::Poisson<pcl::PointNormal> pn;
	pn.setSearchMethod(tree2);
	pn.setInputCloud(cloud_with_normals);
	pn.setThreads(12);										// 多线程加速
	pn.setDepth(6);											// 设置将用于表面重建的树的最大深度
	pn.setMinDepth(2);
	pn.setScale(1.25);										// 设置用于重建的立方体的直径与样本的边界立方体直径的比值
	pn.setSolverDivide(3);									// 设置块高斯-塞德尔求解器用于求解拉普拉斯方程的深度。
	pn.setIsoDivide(6);										// 设置块等表面提取器用于提取等表面的深度
	pn.setSamplesPerNode(10);								// 设置每个八叉树节点上最少采样点数目
	pn.setConfidence(false);								// 设置置信标志，为true时，使用法线向量长度作为置信度信息，false则需要对法线进行归一化处理
	pn.setManifold(false);									// 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	pn.setOutputPolygons(false);							// 设置是否输出为多边形(而不是三角化行进立方体的结果)。
	pn.performReconstruction(triangles);

	cout << "泊松曲面重建算法用时： " << time.toc() / 1000 << " 秒" << endl;

#if ENABLE_DISPLAY
	//----------------------------------结果可视化-----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"泊松曲面重建");
	int v1(0), v2(0);
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);	 // 可视化点云
	viewer->addPolygonMesh(triangles, "my", v2);                 // 可视化模型重建结果
	viewer->setRepresentationToSurfaceForAllActors();            // 网格模型以线框图模式显示
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
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//定义搜索树对象
	tree2->setInputCloud(cloud_with_normals);				//利用有向点云构造tree

	pcl::GridProjection<pcl::PointNormal> gp;
	gp.setInputCloud(cloud_with_normals);
	gp.setSearchMethod(tree2);
	gp.setResolution(0.005);			// 网格分辨率
	gp.setPaddingSize(3);				// 创建的填充单元格的数量
	gp.reconstruct(triangles);			// 曲面重建

	cout << "网格投影曲面重建算法用时： " << time.toc() / 1000 << " 秒" << endl;

#if ENABLE_DISPLAY
	//----------------------------------结果可视化-----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"网格投影曲面重建");
	int v1(0), v2(0);
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);	 // 可视化点云
	viewer->addPolygonMesh(triangles, "my", v2);                 // 可视化模型重建结果
	viewer->setRepresentationToSurfaceForAllActors();            // 网格模型以线框图模式显示
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

	//---------初始化MarchingCubes对象，并设置参数-------
	pcl::MarchingCubes<pcl::PointNormal>* mc;
	mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
	//创建搜索树
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	mc->setInputCloud(cloud_with_normals);
	//设置MarchingCubes对象的参数
	mc->setIsoLevel(0.0f);//该方法设置要提取表面的iso级别
	mc->setGridResolution(50, 50, 50);//用于设置行进立方体网格分辨率
	mc->setPercentageExtendGrid(0.0f);//该参数定义在点云的边框和网格限制之间的网格内应该保留多少自由空间
	//------创建多变形网格，用于存储结果------------
	mc->reconstruct(triangles);//执行重构，结果保存在mesh中

	cout << "移动立方体算法用时： " << time.toc() / 1000 << " 秒" << endl;

#if ENABLE_DISPLAY
	//----------------------------------结果可视化-----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName(u8"移动立方体");
	int v1(0), v2(0);
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);	 // 可视化点云
	viewer->addPolygonMesh(triangles, "my", v2);                 // 可视化模型重建结果
	viewer->setRepresentationToSurfaceForAllActors();            // 网格模型以线框图模式显示
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

	// --------------------------耳朵裁剪三角剖分算法---------------------------
	pcl::EarClipping clipper;
	pcl::PolygonMesh::ConstPtr mesh_aux(mesh);
	clipper.setInputMesh(mesh_aux);

	clipper.process(triangles);

	cout << "耳朵裁剪三角剖分算法用时： " << time.toc() / 1000 << " 秒" << endl;

#if ENABLE_DISPLAY
	//-------------------------------结果可视化----------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	int v1(0), v2(0);
	viewer->setWindowName(u8"耳切算法");
	viewer->createViewPort(0, 0, 0.5, 1, v1);
	viewer->createViewPort(0.5, 0, 1, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);

	viewer->addPolygonMesh(*mesh, "my1", v1);
	viewer->addPolygonMesh(triangles, "my", v2);
	viewer->setRepresentationToSurfaceForAllActors();			//网格模型以线框图模式显示

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
		PCL_ERROR("点云读取失败 \n");
		return (-1);
	} //* the data should be available in cloud

	// 点云预处理
	PreprocessPointCloud(cloud);

	// Normal estimation
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;								//设置法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);			//存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);	//定义kd树指针
	tree->setInputCloud(cloud);																//用cloud构造tree对象
	ne.setInputCloud(cloud);																//为法线估计对象设置输入点云
	ne.setSearchMethod(tree);																//设置搜索方法
	ne.setKSearch(100);																		//设置k邻域搜素的搜索范围
	ne.setNumberOfThreads(4);
	ne.compute(*normals);																	//估计法线
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);//
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);							//连接字段，cloud_with_normals存储有向点云
	//* cloud_with_normals = cloud + normals

	pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
	//GreedyTriangle(*triangles, cloud, cloud_with_normals);					// 贪婪投影三角
	//PossionTriangle(triangles, cloud, cloud_with_normals);				// 泊松重建
	//GridTriangle(triangles, cloud, cloud_with_normals);					// 网格投影曲面重建
	MarchingCubesTriangles(*triangles, cloud, cloud_with_normals);			// 移动立方体
	std::string output_filename = "output_mesh.stl";

	// 保存为 STL 文件
	if (pcl::io::savePolygonFileSTL(output_filename, *triangles) == 0)
	{
		std::cout << "Successfully saved mesh to " << output_filename << std::endl;
	}
	else
	{
		std::cerr << "Failed to save mesh to " << output_filename << std::endl;
		return -1;
	}
	// -----------------------------读取mesh数据-------------------------------
	//pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
	//if (pcl::io::loadPolygonFileVTK("tum_rabbit.vtk", *mesh) == -1)			// tum_rabbit.vtk | sphere.vtk"
	//{
	//	std::cerr << "数据读取失败！！！" << std::endl;
	//}
	//EarClippintTriangles(triangles, mesh);									 // 耳切三角剖分

	size_t polygonNums = triangles->polygons.size();
	std::cout << "面片数：" << polygonNums << std::endl;

	//for (int i = 0; i < polygonNums; ++i)
	//{
	//	std::cout << "polygon" << i << "has" << triangles.polygons[i].vertices.size() << "vertices." << std::endl;
	//}
	
	// 使用Laplacian光顺算法
	//pcl::MeshSmoothingLaplacianVTK smoother;
	//pcl::PolygonMesh smoothed_mesh;
	//smoother.setInputMesh(triangles);
	//smoother.setNumIter(100); // 设置迭代次数
	//smoother.setConvergence(0.001); // 设置收敛标准
	//smoother.setRelaxationFactor(0.1); // 设置松弛因子
	//smoother.process(smoothed_mesh);

	// 光顺不改变面片数
	//polygonNums = smoothed_mesh.polygons.size();
	//std::cout << "光顺后面片数：" << polygonNums << std::endl;

//#if ENABLE_DISPLAY
//	//----------------------------------结果可视化-----------------------------------
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setWindowName(u8"光顺后的模型");
//	//viewer->addPointCloud(cloud, "cloud");
//	viewer->addPolygonMesh(smoothed_mesh, "my");                 // 可视化模型重建结果
//	viewer->setRepresentationToSurfaceForAllActors();            // 网格模型以线框图模式显示
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

	//// 网格简化
	//vtkSmartPointer<vtkPolyData> vtkMesh;
	//pcl::VTKUtils::convertToVTK(smoothed_mesh, vtkMesh);

	//// 使用VTK的DecimatePro 进行简化
	//vtkSmartPointer<vtkDecimatePro> decimate = vtkSmartPointer<vtkDecimatePro>::New();
	//decimate->SetInputData(vtkMesh);
	//decimate->SetTargetReduction(0.5);   // 减少50%的面
	//decimate->Update();

	//// 将简化后的VTK网格转回PCL
	//vtkSmartPointer<vtkPolyData> simplifiedVTKMesh = decimate->GetOutput();
	//pcl::PolygonMesh simplifiedMesh;
	//pcl::VTKUtils::convertToPCL(simplifiedVTKMesh, simplifiedMesh);

	//polygonNums = simplifiedMesh.polygons.size();
	//std::cout << "简化后的面片数：" << polygonNums << std::endl;

//#if ENABLE_DISPLAY
//	//----------------------------------结果可视化-----------------------------------
//	viewer->setWindowName(u8"简化后的面片");
//	int v1(0), v2(0);
//	viewer->createViewPort(0, 0, 0.5, 1, v1);
//	viewer->createViewPort(0.5, 0, 1, 1, v2);
//	viewer->setBackgroundColor(0, 0, 0, v1);
//	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
//	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);	  // 可视化点云
//	viewer->addPolygonMesh(simplifiedMesh, "my", v2);             // 可视化模型重建结果
//	viewer->setRepresentationToSurfaceForAllActors();             // 网格模型以线框图模式显示
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