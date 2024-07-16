#include <pcl/features/normal_3d.h>
#include <pcl/features/ppf.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ppf_registration.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <pcl/registration/icp.h>

using namespace pcl;
using namespace std::chrono_literals;

const Eigen::Vector4f subsampling_leaf_size(1.5f, 1.5f, 1.5f, 0.0f);//�²����������С
constexpr float normal_estimation_search_radius = 5.0f;//���߼��������뾶


int
main(int argc, char** argv)
{
	/// ��ȡ�����ļ�
	PointCloud<PointXYZ>::Ptr cloud_scene(new PointCloud<PointXYZ>());
	if (pcl::io::loadPCDFile("pig_view1.pcd", *cloud_scene) < 0)
	{
		std::cout << "Error loading scene cloud." << std::endl;
		return (-1);
	}
	else
	{
		cout << "cloud_scene ��ȡ�ɹ�" << endl;
	}

	PointCloud<PointXYZ>::Ptr cloud_model(new PointCloud<PointXYZ>());
	if (pcl::io::loadPCDFile("pig_view2.pcd", *cloud_model) < 0)
	{
		std::cout << "Error loading model cloud." << std::endl;
		return (-1);
	}
	else
	{
		cout << "cloud_model ��ȡ�ɹ�" << endl;
	}

	//�������ִ���
	PointCloud<PointNormal>::Ptr cloud_scene_input(new PointCloud<PointNormal>());
	PointCloud<PointXYZ>::Ptr cloud_scene_subsampled(new PointCloud<PointXYZ>());
	//�²����˲�
	VoxelGrid<PointXYZ> subsampling_filter;
	subsampling_filter.setInputCloud(cloud_scene);
	subsampling_filter.setLeafSize(subsampling_leaf_size);
	subsampling_filter.filter(*cloud_scene_subsampled);
	//���㱳������
	PointCloud<Normal>::Ptr cloud_scene_normals(new PointCloud<Normal>());
	NormalEstimation<PointXYZ, Normal> normal_estimation_filter;
	normal_estimation_filter.setInputCloud(cloud_scene_subsampled);
	search::KdTree<PointXYZ>::Ptr search_tree(new search::KdTree<PointXYZ>);
	normal_estimation_filter.setSearchMethod(search_tree);
	normal_estimation_filter.setRadiusSearch(normal_estimation_search_radius);
	normal_estimation_filter.compute(*cloud_scene_normals);
	pcl::concatenateFields(*cloud_scene_subsampled, *cloud_scene_normals, *cloud_scene_input);
	cout << cloud_scene->size() << " down to" << cloud_scene_subsampled->size() << endl;
	//ģ�Ͳ��ִ���
	PointCloud<PointNormal>::Ptr cloud_model_input(new PointCloud<PointNormal>());
	PointCloud<PointXYZ>::Ptr cloud_model_subsampled(new PointCloud<PointXYZ>());
	//�²����˲�
	VoxelGrid<PointXYZ> subsampling_filter2;
	subsampling_filter2.setInputCloud(cloud_model);
	subsampling_filter2.setLeafSize(subsampling_leaf_size);
	subsampling_filter2.filter(*cloud_model_subsampled);
	//���㱳������
	PointCloud<Normal>::Ptr cloud_model_normals(new PointCloud<Normal>());
	NormalEstimation<PointXYZ, Normal> normal_estimation_filter2;
	normal_estimation_filter2.setInputCloud(cloud_model_subsampled);
	search::KdTree<PointXYZ>::Ptr search_tree2(new search::KdTree<PointXYZ>);
	normal_estimation_filter2.setSearchMethod(search_tree2);
	normal_estimation_filter2.setRadiusSearch(normal_estimation_search_radius);
	normal_estimation_filter2.compute(*cloud_model_normals);
	pcl::concatenateFields(*cloud_model_subsampled, *cloud_model_normals, *cloud_model_input);

	cout << cloud_model->size() << " down to" << cloud_model_subsampled->size() << endl;

	// pcl::PointCloud<pcl::PPFSignature>::Ptr cloud_model_ppf = pcl::PointCloud<pcl::PPFSignature>::Ptr(new pcl::PointCloud<pcl::PPFSignature>());
	PointCloud<PPFSignature>::Ptr cloud_model_ppf(new PointCloud<PPFSignature>());
	PPFEstimation<PointNormal, PointNormal, PPFSignature> ppf_estimator;
	ppf_estimator.setInputCloud(cloud_model_input);
	ppf_estimator.setInputNormals(cloud_model_input);
	ppf_estimator.compute(*cloud_model_ppf);//֮ǰһֱ����ָ�뱨���������Ӷ�ά����AGX����
	PPFHashMapSearch::Ptr hashmap_search(new PPFHashMapSearch(2 * float(M_PI) / 20, 0.1f));
	hashmap_search->setInputFeatureCloud(cloud_model_ppf);


	visualization::PCLVisualizer viewer("PPF Object Recognition - Results");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud(cloud_scene);
	viewer.spinOnce(10);
	PCL_INFO("Registering models to scene ...\n");

	//��Դ���ƺ�Ŀ����ƶ�ת��Ϊ�������
	PPFRegistration<PointNormal, PointNormal> ppf_registration;
	// set parameters for the PPF registration procedure
	ppf_registration.setSceneReferencePointSamplingRate(10);
	ppf_registration.setPositionClusteringThreshold(2.0f);
	ppf_registration.setRotationClusteringThreshold(12.0f / 180.0f * float(M_PI));
	ppf_registration.setSearchMethod(hashmap_search);
	ppf_registration.setInputSource(cloud_model_input);
	ppf_registration.setInputTarget(cloud_scene_input);
	//�������
	PointCloud<PointNormal>::Ptr cloud_output_subsampled(new PointCloud<PointNormal>());

	ppf_registration.align(*cloud_output_subsampled);
	//��������Խ����ʣ��������OR������ƣ�  //�����ʵĵط�����������������������������������������������������������
	//�޸�ppf_registration.hpp�е�const auto aux_size = static_cast<std::size_t>(
		//   std::floor(2 * M_PI / search_method_->getAngleDiscretizationStep() + 1));

	//ת������XYZ��ʽ
	PointCloud<PointXYZ>::Ptr cloud_output_subsampled_xyz(new PointCloud<PointXYZ>());
	for (const auto& point : (*cloud_output_subsampled).points)
		cloud_output_subsampled_xyz->points.emplace_back(point.x, point.y, point.z);

	Eigen::Matrix4f mat = ppf_registration.getFinalTransformation();
	std::cout << "PPF �任����" << endl << mat << endl;
	std::cout << "PPF score:" << ppf_registration.getFitnessScore() << endl;
	Eigen::Affine3f final_transformation(mat);
	//���о�ȷ��׼������ICP�㷨
	PointCloud<PointXYZ>::Ptr icp_result(new PointCloud<PointXYZ>());
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//�������׼���ƺ�Ŀ�����
	icp.setInputSource(cloud_model_subsampled);
	icp.setInputTarget(cloud_output_subsampled_xyz);
	//Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance(10);//Ĭ�ϵ�λ����?
	//����������
	icp.setMaximumIterations(1000);
	//���α仯����֮��Ĳ�ֵ
	icp.setTransformationEpsilon(1e-10);
	// �������
	icp.setEuclideanFitnessEpsilon(0.002);
	icp.align(*icp_result, mat);
	Eigen::Matrix4f icp_trans;
	icp_trans = icp.getFinalTransformation();
	std::cout << "icp�任����" << endl << icp_trans << endl;
	std::cout << "icp score:" << icp.getFitnessScore() << endl;
	PointCloud<PointXYZ>::Ptr cloud_output(new PointCloud<PointXYZ>());
	pcl::transformPointCloud(
		*cloud_model, *cloud_output, icp_trans);

	pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> output(cloud_output_subsampled_xyz, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerRandom<PointXYZ> random_color(cloud_output->makeShared());
	viewer.addPointCloud(cloud_output, random_color, "mode_name");
	//viewer.addPointCloud(cloud_output_subsampled_xyz, output, "dsd");
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}
	return 0;
}