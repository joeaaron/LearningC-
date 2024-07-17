#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h> 
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/ppf.h>
#include <pcl/registration/ppf_registration.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/io/vtk_lib_io.h>
#include <boost/thread/thread.hpp>

using namespace std;

#define ENABLE_DISPLAY 1					// 定义一个宏，用于控制显示状态
const double LEAF_SIZE = 1;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

fpfhFeature::Ptr 
compute_fpfh_feature(const pointcloud::Ptr& input_cloud, const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree)
{
	pointnormal::Ptr normals(new pointnormal);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	n.setInputCloud(input_cloud);
	n.setNumberOfThreads(12);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	fpfhFeature::Ptr fpfh(new fpfhFeature);
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
	f.setNumberOfThreads(12);
	f.setInputCloud(input_cloud);
	f.setInputNormals(normals);
	f.setSearchMethod(tree);
	f.setKSearch(30);
	f.compute(*fpfh);
	return fpfh;
}

void 
ComputeFPFH(const pointcloud::Ptr& srcCloud, const pointcloud::Ptr& dstCloud, boost::shared_ptr<pcl::Correspondences>& cru_correspondences)
{
	pcl::console::TicToc time;
	time.tic();

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(srcCloud, tree);
	fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(dstCloud, tree);

	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
	crude_cor_est.setInputSource(source_fpfh);
	crude_cor_est.setInputTarget(target_fpfh);
	crude_cor_est.determineCorrespondences(*cru_correspondences, 0.4);

	Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Ptr trans(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>);
	trans->estimateRigidTransformation(*srcCloud, *dstCloud, *cru_correspondences, Transform);

	cout << "变换矩阵为：\n" << Transform << endl;
	cout << "预对齐算法用时：" << time.toc() / 1000 << " 秒" << endl;

	//进行精确配准，采用ICP算法
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//输入待配准点云和目标点云
	icp.setInputSource(srcCloud);
	icp.setInputTarget(dstCloud);
	//Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance(40);
	//最大迭代次数
	icp.setMaximumIterations(50);
	//两次变化矩阵之间的差值
	icp.setTransformationEpsilon(1e-10);
	// 均方误差
	icp.setEuclideanFitnessEpsilon(0.002);
	icp.align(*icp_result, Transform);
	Eigen::Matrix4f icp_trans;
	icp_trans = icp.getFinalTransformation();
	std::cout << "icp变换矩阵：" << endl << icp_trans << endl;
	std::cout << "icp score:" << icp.getFitnessScore() << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>());

#if ENABLE_DISPLAY
	/*pcl::transformPointCloud(*srcCloud, *srcCloud, Transform);*/
	pcl::transformPointCloud(
		*srcCloud, *cloud_output, icp_trans);
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer(u8"显示点云"));
	viewer->setBackgroundColor(255, 255, 255);
	// 对目标点云着色可视化 (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(dstCloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(dstCloud, target_color, "target cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
	// 对源点云着色可视化 (green).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>input_color(srcCloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_output, input_color, "input cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input cloud");
	//对应关系可视化
	//viewer->addCorrespondences<pcl::PointXYZ>(cloud_output, dstCloud, *cru_correspondences, "correspondence");
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
}

void 
ComputePPF(const pointcloud::Ptr& srcCloud, 
	const pointcloud::Ptr& dstCloud,
	boost::shared_ptr<pcl::Correspondences>& cru_correspondences)
{
	// 1. 使用法线估计方法计算法线
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>());
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>());

	normal_estimation.setInputCloud(srcCloud);
	normal_estimation.setSearchMethod(tree);
	normal_estimation.setKSearch(30);
	normal_estimation.setNumberOfThreads(12);
	normal_estimation.compute(*source_normals);

	normal_estimation.setInputCloud(dstCloud);
	normal_estimation.compute(*target_normals);

	pcl::PointCloud <pcl::PointNormal>::Ptr model_keypoints_with_normals = pcl::PointCloud <pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>());
	pcl::PointCloud<pcl::PointNormal>::Ptr scene_keypoints_with_normals = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());

	pcl::concatenateFields(*srcCloud, *source_normals, *model_keypoints_with_normals);
	pcl::concatenateFields(*dstCloud, *target_normals, *scene_keypoints_with_normals);

	// 2. 计算点对特征（PPF）
	pcl::PointCloud<pcl::PPFSignature>::Ptr source_ppf = pcl::PointCloud<pcl::PPFSignature>::Ptr(new pcl::PointCloud<pcl::PPFSignature>());
	pcl::PPFEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PPFSignature> ppf_estimation;

	ppf_estimation.setInputCloud(model_keypoints_with_normals);
	ppf_estimation.setInputNormals(model_keypoints_with_normals);
	ppf_estimation.compute(*source_ppf);

	// 3. 使用哈希表进行特征匹配
	pcl::PPFHashMapSearch::Ptr hashmap_search(new pcl::PPFHashMapSearch(1.0f / 5.0f));
	hashmap_search->setInputFeatureCloud(source_ppf);

	// 4. 执行配准
	pcl::PPFRegistration<pcl::PointNormal, pcl::PointNormal> ppf_registration;
	ppf_registration.setSearchMethod(hashmap_search);
	ppf_registration.setInputSource(model_keypoints_with_normals);
	ppf_registration.setInputTarget(scene_keypoints_with_normals);

	pcl::PointCloud<pcl::PointXYZ> cloud_output;
	Eigen::Matrix4f transformation;

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_output_subsampled(new pcl::PointCloud<pcl::PointNormal>());
	ppf_registration.align(*cloud_output_subsampled);

	std::cout << "Transformation Matrix: \n" << transformation << std::endl;
}

bool Mesh2CloudPCL(pcl::PointCloud<pcl::PointXYZ>& cloudOut,
	const pcl::PolygonMesh& mesh)
{
	pcl::fromPCLPointCloud2(mesh.cloud, cloudOut);
	return true;
}

int
main(int argc, char** argv)
{
	/*pointcloud::Ptr source_cloud(new pointcloud);
	pointcloud::Ptr target_cloud(new pointcloud);*/

	/*pcl::io::loadPCDFile<pcl::PointXYZ>("pig_view1.pcd", *source_cloud);
	pcl::io::loadPCDFile<pcl::PointXYZ>("pig_view2.pcd", *target_cloud);*/


	pcl::PolygonMesh mesh;
	if (pcl::io::loadPolygonFileSTL("model.stl", mesh) == -1)		//Prismatic002.stl model.STL
	{
		PCL_ERROR("STL读取失败 \n");
		return (-1);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudModel(new pcl::PointCloud<pcl::PointXYZ>);
	Mesh2CloudPCL(*cloudModel, mesh);

	//Load scene
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudScene(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("scene.pcd", *cloudScene) == -1) 
	{
		PCL_ERROR("Couldn't read file\n");
		return (-1);
	}

	cout << "点云点数：" << cloudScene->points.size() << endl;
	boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);

	//pcl::Indices indices1;
	//for (int i = 0; i < cloudModel->points.size(); i += 2)
	//{
	//	indices1.push_back(i);
	//}
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudModelFiltered(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::copyPointCloud(*cloudModel, indices1, *cloudModelFiltered);

	//pcl::Indices indices2;
	//for (int i = 0; i < cloudScene->points.size(); i += 1)
	//{
	//	indices2.push_back(i);
	//}
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSceneFiltered(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::copyPointCloud(*cloudScene, indices2, *cloudSceneFiltered);
	//-----------------------------------------
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
	vg.setDownsampleAllData(false);

	vg.setInputCloud(cloudModel);
	vg.filter(*cloudModel);

	vg.setInputCloud(cloudScene);
	vg.filter(*cloudScene);

	//// 计算质心
	//Eigen::Vector4f centroid;
	//pcl::compute3DCentroid(*cloudModel, centroid);
	//std::cout << "质心: " << centroid.transpose() << std::endl;

	//// 平移矩阵，将质心移动到原点
	//Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	//transform.translation() << -centroid[0], -centroid[1], -centroid[2];

	//// 变换点云
	//pcl::transformPointCloud(*cloudModel, *cloudModel, transform);

	//pcl::compute3DCentroid(*cloudScene, centroid);
	//std::cout << "质心: " << centroid.transpose() << std::endl;

	//// 平移矩阵，将质心移动到原点
	//transform = Eigen::Affine3f::Identity();
	//transform.translation() << -centroid[0], -centroid[1], -centroid[2];

	//// 变换点云
	//pcl::transformPointCloud(*cloudScene, *cloudScene, transform);

	ComputeFPFH(cloudModel, cloudScene, cru_correspondences);
	//ComputePPF(cloudModel, cloudScene, cru_correspondences);
	return 0;
}

