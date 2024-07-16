#include "InitialAlign.h"
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h> 
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::FPFHSignature33> FpfhFeature;
const double LEAF_SIZE = 1.0;

namespace
{
	template <typename PointT>
	void Mesh2Cloud(pcl::PointCloud<PointT>& cloudOut,
		const pcl::PolygonMesh& mesh)
	{
		pcl::fromPCLPointCloud2(mesh.cloud, cloudOut);
		return;
	}

	FpfhFeature::Ptr ComputeFeature(const PclCloudPtr_Point& cloudIn,
		const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree)
	{
		// 计算法向量
		pcl::PointCloud<pcl::Normal>::Ptr pNormal(new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(cloudIn);
		ne.setNumberOfThreads(12);
		ne.setSearchMethod(tree);
		ne.setKSearch(20);
		ne.compute(*pNormal);

		// 计算FPFH特征
		FpfhFeature::Ptr fpfh(new FpfhFeature);
		pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fp;
		fp.setNumberOfThreads(12);
		fp.setInputCloud(cloudIn);
		fp.setInputNormals(pNormal);
		fp.setSearchMethod(tree);
		fp.setKSearch(30);
		fp.compute(*fpfh);

		return fpfh;
	}

	bool ComputeFPFH(Eigen::Matrix4d& transform,
		const PclCloudPtr_Point& pCloudModel,
		const PclCloudPtr_Point& pCloudScene)
	{
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		FpfhFeature::Ptr modelFpfh = ComputeFeature(pCloudModel, tree);
		FpfhFeature::Ptr sceneFpfh = ComputeFeature(pCloudScene, tree);

		pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
		std::shared_ptr<pcl::Correspondences> corresponds(new pcl::Correspondences);

		crude_cor_est.setInputSource(modelFpfh);
		crude_cor_est.setInputTarget(sceneFpfh);
		crude_cor_est.determineCorrespondences(*corresponds, 0.4);

		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, double>::Ptr trans(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, double>);
		trans->estimateRigidTransformation(*pCloudModel, *pCloudScene, *corresponds, transform);

		return true;
	}
}

PCL_InitialAlign::PCL_InitialAlign()
	: m_eMessage(em3DCloudSampleMessage::eMessageOK)
	, m_eInitialAlignType(em3DCloudInitialAlignType::eFast)
{
	m_nThreadNum = static_cast<int>(omp_get_num_procs() * 0.75);
	m_transform = Eigen::Matrix4d::Identity();
}

PCL_InitialAlign::~PCL_InitialAlign()
{

}

void PCL_InitialAlign::SetInitialAlignType(const eExtraPcl_InitialAlignType& eInitialAlignType)
{
	m_eInitialAlignType = eInitialAlignType;
}

Eigen::Matrix4d PCL_InitialAlign::GetTransform() const
{
	return m_transform;
}

template <typename PointT>
bool PCL_InitialAlign::Execute(const pcl::PolygonMesh& meshIn, const std::shared_ptr<pcl::PointCloud<PointT>>& pCloudIn)
{
	if (pCloudIn == nullptr)
	{
		return false;
	}

	bool bSucceed = false;

	// STL转点云
	PclCloudPtr_Point cloudModel(new PclCloud_Point);
	Mesh2Cloud(*cloudModel, meshIn);

	// 下采样
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
	vg.setDownsampleAllData(false);

	vg.setInputCloud(cloudModel);
	vg.filter(*cloudModel);

	vg.setInputCloud(pCloudIn);
	vg.filter(*pCloudIn);

	switch (m_eInitialAlignType)
	{
	case em3DCloudInitialAlignType::eFast:
		bSucceed = ComputeFPFH(m_transform, cloudModel, pCloudIn);
		break;
	default:
		break;
	}

	return bSucceed;
}


template bool PCL_InitialAlign::Execute(const pcl::PolygonMesh& meshIn, const std::shared_ptr<PclCloud_Point>& pCloudIn);
