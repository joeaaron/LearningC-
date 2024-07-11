#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h> 
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
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


void ComputeFPFH(pointcloud::Ptr& srcCloud, pointcloud::Ptr& dstCloud, boost::shared_ptr<pcl::Correspondences>& cru_correspondences)
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
	cout << "FPFH配准算法用时： " << time.toc() / 1000 << " 秒" << endl;
}

int
main(int argc, char** argv)
{
	pointcloud::Ptr source_cloud(new pointcloud);
	pointcloud::Ptr target_cloud(new pointcloud);
	pcl::io::loadPCDFile<pcl::PointXYZ>("pig_view1.pcd", *source_cloud);
	pcl::io::loadPCDFile<pcl::PointXYZ>("pig_view2.pcd", *target_cloud);

	cout << "点云点数：" << source_cloud->points.size() << endl;
	boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);

	ComputeFPFH(source_cloud, target_cloud, cru_correspondences);
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer(u8"显示点云"));
	viewer->setBackgroundColor(255, 255, 255);
	// 对目标点云着色可视化 (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target_cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
	// 对源点云着色可视化 (green).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>input_color(source_cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(source_cloud, input_color, "input cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input cloud");
	//对应关系可视化
	viewer->addCorrespondences<pcl::PointXYZ>(source_cloud, target_cloud, *cru_correspondences, "correspondence");
	//viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}

