#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/console/time.h> 
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkTriangle.h>
#include <Eigen/Dense>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/ppf.h>
#include <pcl/registration/ppf_registration.h>
#include "MyPPFRegistration.hpp"

const int SAMPLE_POINTS = 1000000;
const float MODEL_LEAF_SIZE = 1.0f;
const float SCENE_LEAF_SIZE = 0.001f;

bool bCalNormal = true;
bool bCalColor = false;

#define ENABLE_DISPLAY 1				// 定义一个宏，用于控制显示状态

using namespace Eigen;
double UniformDeviate(int seed)
{
	double ran = seed * (1.0 / (RAND_MAX + 1.0));
	return ran;
}

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
	grid.setLeafSize(MODEL_LEAF_SIZE, MODEL_LEAF_SIZE, MODEL_LEAF_SIZE);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	grid.filter(*voxel_cloud);

	pcl::copyPointCloud(*voxel_cloud, cloudOut);

	return true;
}

void HPR(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in, std::vector<float> camera_pos, int param, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) // Hidden Point Removal
{
	int dim = 3;
	int numPts = cloud_in->size();

	ArrayXXd p(numPts, dim);
	for (int i = 0; i < cloud_in->size(); ++i)
	{
		p(i, 0) = cloud_in->at(i).x;
		p(i, 1) = cloud_in->at(i).y;
		p(i, 2) = cloud_in->at(i).z;
	}

	ArrayXd C(3); C << camera_pos[0], camera_pos[1], camera_pos[2];
	p = p - C.transpose().replicate(numPts, 1);

	ArrayXd normp = (p * p).rowwise().sum().sqrt();
	ArrayXd maxNormp(1); maxNormp << normp.maxCoeff() * pow(10, param);
	ArrayXd R = maxNormp.replicate(numPts, 1);

	ArrayXXd P = p + 2 * (R - normp).replicate(1, dim) * p / normp.replicate(1, dim);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_P(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < P.rows(); ++i)
	{
		pcl::PointXYZ point;
		point.x = P(i, 0);
		point.y = P(i, 1);
		point.z = P(i, 2);
		cloud_P->push_back(point);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::Vertices> polygons;
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setDimension(3);
	chull.setInputCloud(cloud_P);
	chull.reconstruct(*cloud_hull, polygons);

	pcl::PointIndices::Ptr indices(new pcl::PointIndices);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_P);

	for (int i = 0; i < cloud_hull->size(); ++i)
	{
		int K = 1;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		if (kdtree.nearestKSearch(cloud_hull->at(i), K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			indices->indices.push_back(pointIdxNKNSearch[0]);
		}
	}

	// Extract the inliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_in);
	extract.setIndices(indices);
	extract.setNegative(false);
	extract.filter(*cloud_out);
}

double ComputeCloudDiameter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.005f, 0.005f, 0.005f);
	vg.setDownsampleAllData(false);
	vg.filter(*cloud_downsampled);

	double diameter_sqr = 0;
	for (size_t i = 0; i < cloud_downsampled->points.size(); i += 10)
	{
		for (size_t j = 0; j < cloud_downsampled->points.size(); j += 10)
		{
			if (i == j)
				continue;
			double distance_sqr = (cloud_downsampled->points[i].x - cloud_downsampled->points[j].x) * (cloud_downsampled->points[i].x - cloud_downsampled->points[j].x)
				+ (cloud_downsampled->points[i].y - cloud_downsampled->points[j].y) * (cloud_downsampled->points[i].y - cloud_downsampled->points[j].y)
				+ (cloud_downsampled->points[i].z - cloud_downsampled->points[j].z) * (cloud_downsampled->points[i].z - cloud_downsampled->points[j].z);
			if (distance_sqr > diameter_sqr)
			{
				diameter_sqr = distance_sqr;
			}
		}
	}
	return sqrt(diameter_sqr);
}

void RemoveBg(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, double dist_threshold, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	// Identify the table
	pcl::PointIndices::Ptr sacs_inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr sacs_coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZ> sacs;
	sacs.setOptimizeCoefficients(true);
	sacs.setModelType(pcl::SACMODEL_PLANE);
	sacs.setMethodType(pcl::SAC_RANSAC);
	sacs.setMaxIterations(900);//900
	sacs.setDistanceThreshold(dist_threshold);//16mm
	sacs.setInputCloud(cloud_in);
	sacs.segment(*sacs_inliers, *sacs_coefficients);

	// Remove the table
	pcl::ExtractIndices<pcl::PointXYZ> ei;
	ei.setInputCloud(cloud_in);
	ei.setIndices(sacs_inliers);
	ei.setNegative(true);
	ei.filter(*cloud_out);
}

void StatisticalOutlinerRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
	int numNeighbors, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out) 
{
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_in);
	sor.setMeanK(numNeighbors);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_out);
}

int main()
{
	//Load model
	pcl::PolygonMesh mesh;
	if (pcl::io::loadPolygonFileSTL("Prismatic002.stl", mesh) == -1)		//Prismatic002.stl model.STL
	{
		PCL_ERROR("STL读取失败 \n");
		return (-1);
	}

	//Load scene
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudScene(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("Prismatic002.pcd", *cloudScene) == -1) //* load the file Prismatic002.pcd scene.pcd
	{
		PCL_ERROR("Couldn't read file\n");
		return (-1);
	}

	pcl::console::TicToc time;
	time.tic();

	std::cout << "Step 1: Load STL file and perform point sampling from each view" << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSampled(new pcl::PointCloud<pcl::PointXYZ>());
	Mesh2Cloud(*cloudSampled, mesh);

	//---- Calculate point cloud from 6 views and combine ------
	std::vector<std::vector<float>> camera_pos(6);
	pcl::PointXYZ minPt, maxPt, avgPt;

	pcl::getMinMax3D(*cloudSampled, minPt, maxPt);
	avgPt.x = (minPt.x + maxPt.x) / 2;
	avgPt.y = (minPt.y + maxPt.y) / 2;
	avgPt.z = (minPt.z + maxPt.z) / 2;

	float cube_length = std::max(maxPt.x - minPt.x, std::max(maxPt.y - minPt.y, maxPt.z - minPt.z));

	minPt.x = avgPt.x - cube_length;
	minPt.y = avgPt.y - cube_length;
	minPt.z = avgPt.z - cube_length;
	maxPt.x = avgPt.x + cube_length;
	maxPt.y = avgPt.y + cube_length;
	maxPt.z = avgPt.z + cube_length;

	camera_pos[0] = { avgPt.x, minPt.y, avgPt.z };
	camera_pos[1] = { maxPt.x, avgPt.y, avgPt.z };
	camera_pos[2] = { avgPt.x, maxPt.y, avgPt.z };
	camera_pos[3] = { minPt.x, avgPt.y, avgPt.z };
	camera_pos[4] = { avgPt.x, avgPt.y, maxPt.z };
	camera_pos[5] = { avgPt.x, avgPt.y, minPt.z };

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudModel = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	for (int i = 0; i < static_cast<int>(camera_pos.size()); ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZHPR(new pcl::PointCloud<pcl::PointXYZ>());
		HPR(cloudSampled, camera_pos[i], 3, cloudXYZHPR);

		*cloudModel += *cloudXYZHPR;
	}

	// ------- Centering the model ----------------
	Eigen::Vector3d sum_of_pos = Eigen::Vector3d::Zero();
	for (const auto& p : *(cloudModel)) sum_of_pos += p.getVector3fMap().cast<double>();

	Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
	transform.topRightCorner<3, 1>() = -sum_of_pos / cloudModel->size();

	pcl::transformPointCloud(*cloudModel, *cloudModel, transform);
	pcl::transformPointCloud(*cloudModel, *cloudModel, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0.7071, 0, -0.7071, 0));

	std::cout << "Step 2: Prepare Point Pair Feature descriptors of model\n";
	//Model diameter is the furthest distance from any 2 points of the cloud
	double modelDiameter = ComputeCloudDiameter(cloudModel);
	//We set the params based on the diameter to have general purpose
	double t_sampling = 0.04;
	float samp_rad = t_sampling * modelDiameter;
	float norm_rad = 2 * samp_rad;
	double dVoxel = samp_rad;

	//Voxel grid filter
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloudModel);
	vg.setLeafSize(samp_rad, samp_rad, samp_rad);
	vg.setDownsampleAllData(false);
	vg.filter(*model_keypoints);

	// Calculate all the normals of the entire surface
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud <pcl::PointNormal>::Ptr model_keypoints_with_normals = pcl::PointCloud <pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>());

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	ne.setInputCloud(model_keypoints);
	ne.setSearchSurface(cloudModel);
	ne.setNumberOfThreads(4);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(norm_rad);
	ne.compute(*normals);

	pcl::concatenateFields(*model_keypoints, *normals, *model_keypoints_with_normals);

	//Calculate PPF Descriptor of the model
	pcl::PointCloud<pcl::PPFSignature>::Ptr descriptors_PPF = pcl::PointCloud<pcl::PPFSignature>::Ptr(new pcl::PointCloud<pcl::PPFSignature>());
	pcl::PPFEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PPFSignature> ppf_estimator;
	ppf_estimator.setInputCloud(model_keypoints_with_normals);
	ppf_estimator.setInputNormals(model_keypoints_with_normals);
	ppf_estimator.compute(*descriptors_PPF);

	float angle_discretization_step = 12.0f / 180.0f * float(M_PI);
	float distance_discretization_step = 0.005f;
	std::shared_ptr<pcl::MyPPFHashMapSearch> ppf_hashmap_search = std::make_shared<pcl::MyPPFHashMapSearch>(angle_discretization_step, distance_discretization_step);
	ppf_hashmap_search->setInputFeatureCloud(descriptors_PPF);

	std::cout << "Step 3: Capture Point Cloud\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr captured_scene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*cloudScene, *captured_scene);

	std::cout << "Step 4: Remove background\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedScene = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	RemoveBg(captured_scene, 0.005, segmentedScene);

	if (segmentedScene->size() == 0)
	{
		std::cout << "No point left in scene. Skipping this frame ..." << std::endl;
		return -1;
	}

	std::cout << "Step 5: Voxel Grid and Remove Outliner:" << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::VoxelGrid<pcl::PointXYZ> vg_;
	vg_.setInputCloud(segmentedScene);
	vg_.setLeafSize(SCENE_LEAF_SIZE, SCENE_LEAF_SIZE, SCENE_LEAF_SIZE);
	vg_.setDownsampleAllData(true);
	vg_.filter(*scene_keypoints);
	StatisticalOutlinerRemoval(scene_keypoints, 50, scene_keypoints);// 50 k-neighbors noise removal

	vg_.setInputCloud(scene_keypoints);
	vg_.setLeafSize(samp_rad, samp_rad, samp_rad);
	vg_.setDownsampleAllData(true);
	vg_.filter(*scene_keypoints);

	std::cout << "Step 6: Calculate normals (using unfilter cloud as surface)\n";
	pcl::PointCloud<pcl::Normal>::Ptr sceneNormals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr scene_keypoints_with_normals = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> sceneNe;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr sceneTree(new pcl::search::KdTree<pcl::PointXYZ>);
	// Calculate all the normals of the entire surface
	sceneNe.setInputCloud(scene_keypoints);
	sceneNe.setSearchSurface(segmentedScene);
	sceneNe.setNumberOfThreads(4);
	sceneNe.setSearchMethod(sceneTree);
	sceneNe.setRadiusSearch(norm_rad);
	sceneNe.compute(*sceneNormals);

	pcl::concatenateFields(*scene_keypoints, *sceneNormals, *scene_keypoints_with_normals);

	std::cout << "Step 7: PPF\n";
	pcl::MyPPFRegistration<pcl::PointNormal, pcl::PointNormal> ppf_registration;
	// set parameters for the PPF registration procedure
	int scene_reference_point_sampling_rate = 10;
	int scene_referred_point_sampling_rate = 5;
	ppf_registration.setSceneReferencePointSamplingRate(scene_reference_point_sampling_rate);
	ppf_registration.setSceneReferredPointSamplingRate(scene_referred_point_sampling_rate);
	ppf_registration.setLvoxel(dVoxel);
	ppf_registration.setSearchMethod(ppf_hashmap_search);
	ppf_registration.setInputSource(model_keypoints_with_normals);
	ppf_registration.setInputTarget(scene_keypoints_with_normals);

	typename pcl::MyPPFRegistration<pcl::PointNormal, pcl::PointNormal>::PoseWithVotesList results;
	ppf_registration.computeFinalPoses(results);

	typename pcl::MyPPFRegistration<pcl::PointNormal, pcl::PointNormal>::PoseWithVotesList verified_results;
	ppf_registration.verifyPoses(results, verified_results);
	for (size_t results_i = 0; results_i < verified_results.size(); ++results_i)
	{
		// Generates clouds for each instances found
		pcl::PointCloud<pcl::PointXYZ>::Ptr instance(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::transformPointCloud(*model_keypoints, *model_keypoints, verified_results[results_i].pose);

		Eigen::Matrix4f transform = verified_results[results_i].pose.matrix();
	}
	cout << "PPF算法用时： " << time.toc() / 1000 << " 秒" << endl;

#if ENABLE_DISPLAY
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer(u8"显示点云"));
	viewer->setBackgroundColor(255, 255, 255);
	// 对目标点云着色可视化 (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(scene_keypoints, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(scene_keypoints, target_color, "target cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
	// 对源点云着色可视化 (green).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>input_color(model_keypoints, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(model_keypoints, input_color, "input cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input cloud");
	//对应关系可视化
	//viewer->addCorrespondences<pcl::PointXYZ>(model_keypoints, scene_keypoints, *cru_correspondences, "correspondence");
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	////-----------------结果显示---------------------
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Mesh to cloud"));

	//// 分2个窗口进行显示
	//int v1(0);
	//viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	//viewer->setBackgroundColor(0.3, 0.3, 0.3, v1);
	//viewer->addText("cloud", 10, 10, "v1_text", v1);

	//int v2(0);
	//viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
	//viewer->setBackgroundColor(0.5, 0.5, 0.5, v2);
	//viewer->addText("cloudUnm", 10, 10, "v2_text", v2);

	//viewer->addPointCloud<pcl::PointXYZ>(cloudModel, "cloudModel", v1);
	//viewer->addPointCloud<pcl::PointXYZ>(scene_keypoints, "cloudScene", v2);

	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloudModel", v1);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloudScene", v2);

	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(1000);
	//}
#endif
	return 0;
}