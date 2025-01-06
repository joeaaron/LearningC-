#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/gp3.h>
#include <chrono>

using namespace std;
#define ENABLE_DISPLAY 0		// ����һ���꣬���ڿ�����ʾ״̬

typedef pcl::PointXYZ PointT;

void VisualizedCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr& pSmoothedCloud, const pcl::PointCloud<PointT>::Ptr& pCloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("���Ʊ߽���ȡ"));

	int v1(0);
	MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
	MView->addText("Raw cloud", 10, 10, "v1_text", v1);

	int v2(0);
	MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
	MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
	MView->addText("Smoothed cloud", 10, 10, "v2_text", v2);

	MView->addPointCloud<PointT>(pCloud, "Raw cloud", v1);
	MView->addPointCloud<pcl::PointNormal>(pSmoothedCloud, "Smoothed cloud", v2);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Raw cloud", v1);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "Smoothed cloud", v2);
	MView->addCoordinateSystem(1.0);
	MView->initCameraParameters();

	MView->spin();
}

static void CalcNormal(pcl::PointCloud<pcl::Normal>::Ptr& pNormal, const pcl::PointCloud<PointT>::Ptr& pCloud)
{
	// ����KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(pCloud);

	// ʹ��NormalEstimation���㷨����
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(pCloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(2); // ��������뾶
	ne.setNumberOfThreads(4);

	ne.compute(*pNormal);
}

static void MarchingCubesTriangles(pcl::PolygonMesh& triangles,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals)
{

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

void GreedyTriangle(pcl::PolygonMesh& triangles,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals)
{

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

// Moving Least Squares��MLS��ƽ��
void MovingLeastSquares(pcl::PointCloud<PointT>::Ptr& pSmoothedCloud, const pcl::PointCloud<PointT>::Ptr& pCloud)
{	
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls_omp; // OMP�汾��MLS
	mls_omp.setInputCloud(pCloud);
	mls_omp.setSearchMethod(pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());  // ʹ��KdTree��������
	mls_omp.setSearchRadius(6);		    // ���������뾶
	//mls_omp.setPolynomialFit(true);   // ʹ�ö���ʽ���
	mls_omp.setPolynomialOrder(2);		// ���ö���ʽ�Ľ���
	//mls_omp.setPolynomialFit(false);  // ����Ϊfalse���� ���� smooth
	mls_omp.setSqrGaussParam(0.01);		// ��˹�����Ĳ�����Ӱ��ƽ���̶�
	mls_omp.setNumberOfThreads(4);		// ����ʹ�õ��߳���������CPU����������
	mls_omp.process(*pSmoothedCloud);
}

// ���������ƽ������ƽ���㷨
void NeighborBasedNormal(pcl::PointCloud<pcl::PointNormal>::Ptr& pSmoothedCloud, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals, const pcl::PointCloud<PointT>::Ptr& pCloud)
{
	// ����������
	pcl::PointCloud<pcl::Normal>::Ptr pNormal(new pcl::PointCloud<pcl::Normal>);
	CalcNormal(pNormal, pCloud);

	// �������뷨�����ϲ�
	pcl::concatenateFields(*pCloud, *pNormal, *cloud_with_normals);

	// ��������ܶ�
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(pCloud);
	std::vector<float> point_densities(pCloud->points.size(), 0.0f);
	int k = 10;  // �������
	for (size_t i = 0; i < pCloud->points.size(); ++i) {
		std::vector<int> pointIdxNKNSearch(k);
		std::vector<float> pointNKNSquaredDistance(k);
		if (kdtree.nearestKSearch(pCloud->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
			float average_distance = 0.0f;
			for (size_t j = 1; j < pointIdxNKNSearch.size(); ++j) {  // ������һ���㣨����
				average_distance += std::sqrt(pointNKNSquaredDistance[j]);
			}
			average_distance /= (k - 1);
			point_densities[i] = 1.0f / average_distance;  // �ܶ� = ����ĵ���
		}
	}

	pcl::KdTreeFLANN<pcl::PointNormal> kdtree_normals;
	kdtree_normals.setInputCloud(cloud_with_normals);

	float alpha = 2.0f;  // �����뾶����ϵ��

	for (size_t i = 0; i < cloud_with_normals->points.size(); ++i) {
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		float radius = alpha / point_densities[i];  // ��̬�뾶
		// ��������
		if (kdtree_normals.radiusSearch(cloud_with_normals->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			Eigen::Vector3f normal_sum(0.0, 0.0, 0.0);

			// �ۼӷ�����
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
				normal_sum += cloud_with_normals->points[pointIdxRadiusSearch[j]].getNormalVector3fMap();
			}

			// ƽ������һ��
			normal_sum.normalize();
			pcl::PointNormal point = cloud_with_normals->points[i];
			point.normal_x = normal_sum[0];
			point.normal_y = normal_sum[1];
			point.normal_z = normal_sum[2];

			pSmoothedCloud->points.push_back(point);
		}
	}

	pSmoothedCloud->width = pSmoothedCloud->points.size();
	pSmoothedCloud->height = 1;
	pSmoothedCloud->is_dense = true;

}

// ���㷨�����ı�׼��
static void ComputeNormalVariance(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals)
{
	// ���㷨�����ľ�ֵ
	Eigen::Vector3f normal_mean(0.0f, 0.0f, 0.0f);
	for (size_t i = 0; i < cloud_with_normals->points.size(); ++i) {
		Eigen::Vector3f normal = cloud_with_normals->points[i].getNormalVector3fMap();
		// ��鷨�����Ƿ����NaN
		if (normal.hasNaN()) {
			//std::cerr << "Warning: NaN detected in normal vector at index " << i << std::endl;
			continue;  // ��������NaN�ķ�����
		}
		normal_mean += normal;
	}
	normal_mean /= cloud_with_normals->points.size();

	// ���㷨�����ķ���
	float variance = 0.0f;
	for (size_t i = 0; i < cloud_with_normals->points.size(); ++i) {
		Eigen::Vector3f normal = cloud_with_normals->points[i].getNormalVector3fMap();
		if (normal.hasNaN()) {
			//std::cerr << "Warning: NaN detected in normal vector at index " << i << std::endl;
			continue;  // ��������NaN�ķ�����
		}
		float diff = (normal - normal_mean).norm();  // ���㷨�������ֵ��ŷ�Ͼ���
		variance += diff * diff;
	}

	// ����
	variance /= cloud_with_normals->points.size();
	// ��׼��
	float standard_deviation = std::sqrt(variance);

	// ����������ı�׼���Ϊһ���Ե�ָ��
	std::cout << "Normal Vector Standard Deviation: " << standard_deviation << std::endl;
}

static int ReadFile(std::vector<Eigen::Vector3d>& vNoramls, std::string filePath)
{
	// ���ļ�
	std::ifstream file(filePath);
	if (!file.is_open()) {
		std::cerr << "Unable to open file " << filePath << std::endl;
		return -1;
	}

	std::string line;
	// ���ж�ȡ�ļ�
	while (std::getline(file, line)) {
		std::istringstream ss(line);
		double x, y, z, nx, ny, nz;

		// ��ȡÿһ���е�ֵ
		ss >> x >> y >> z >> nx >> ny >> nz;

		// �������е�ֵ����Eigen::Vector3d����
		Eigen::Vector3d point(nx, ny, nz);
		vNoramls.push_back(point);
	}

	// �ر��ļ�
	file.close();

	return 0;
	//// ��ӡ��ȡ��ֵ
	//for (const auto& point : vNoramls) {
	//	std::cout << "Vector: " << point.transpose() << std::endl;
	//}

}

//static void CalcNormalVariance(const std::vector<Eigen::Vector3d>& points)
//{
//	// ���㷨�����ľ�ֵ
//	Eigen::Vector3d mean(0, 0, 0);
//	for (const auto& point : points) {
//		mean += point;
//	}
//	mean /= points.size();
//
//	std::cout << "Mean of the normal vectors: " << mean.transpose() << std::endl;
//
//	// ���㷨�����ķ���
//	//Eigen::Vector3d variance(0, 0, 0);
//	double variance = 0.0;
//	for (const auto& point : points) {
//		double diff = (point - mean).norm();
//		//variance += diff.cwiseProduct(diff);
//		variance += diff * diff;
//	}
//
//	variance /= points.size();
//	std::cout << "Standard deviation of the normal vectors: " << std::sqrt(variance) << std::endl;
//}

static void CalcNormalVariance(const std::vector<Eigen::Vector3d>& points)
{
	// ���㷨�����ľ�ֵ
	Eigen::Vector3d mean(0, 0, 0);
	for (const auto& point : points) {
		mean += point;
	}
	mean /= points.size();

	std::cout << "Mean of the normal vectors: " << mean.transpose() << std::endl;

	// ���㷨�����ķ���
	double variance = 0.0;
	std::vector<double> angle_differences;  // �����洢�ǶȲ���
	for (size_t i = 0; i < points.size(); ++i) {
		const Eigen::Vector3d& point = points[i];
		double diff = (point - mean).norm();
		variance += diff * diff;

		// �������ڷ�����֮��ļнǲ���
		if (i > 0) {
			const Eigen::Vector3d& prev_point = points[i - 1];
			// ����н�
			double cos_angle = point.dot(prev_point) / (point.norm() * prev_point.norm());
			// ����cosֵ��Χ��[-1, 1]֮�䣬������ֵ���
			cos_angle = std::clamp(cos_angle, -1.0, 1.0);
			double angle = std::acos(cos_angle);  // ����нǣ����ȣ�
			angle_differences.push_back(angle);
		}
	}

	// �����������׼�
	variance /= points.size();
	double normal_standard_deviation = std::sqrt(variance);

	// ����ǶȲ���ı�׼��
	double angle_variance = 0.0;
	for (double angle : angle_differences) {
		angle_variance += angle * angle;
	}
	angle_variance /= angle_differences.size();
	double angle_standard_deviation = std::sqrt(angle_variance);

	// ����������ı�׼��ͷ������нǵı�׼��
	std::cout << "Standard Deviation of the normal vectors: " << normal_standard_deviation << std::endl;
	std::cout << "Standard Deviation of the normal vector angles: " << angle_standard_deviation << " radians" << std::endl;
}

static void ComputeNormalVarianceAndAngle(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals)
{
	// ���㷨�����ľ�ֵ
	Eigen::Vector3f normal_mean(0.0f, 0.0f, 0.0f);
	for (size_t i = 0; i < cloud_with_normals->points.size(); ++i) {
		Eigen::Vector3f normal = cloud_with_normals->points[i].getNormalVector3fMap();
		if (normal.hasNaN()) {
			continue;  // ��������NaN�ķ�����
		}
		normal_mean += normal;
	}
	normal_mean /= cloud_with_normals->points.size();

	// ���㷨�����ķ���
	float variance = 0.0f;
	std::vector<float> angle_differences;
	for (size_t i = 0; i < cloud_with_normals->points.size(); ++i) {
		Eigen::Vector3f normal = cloud_with_normals->points[i].getNormalVector3fMap();
		if (normal.hasNaN()) {
			continue;  // ��������NaN�ķ�����
		}
		float diff = (normal - normal_mean).norm();  // ���㷨�������ֵ��ŷ�Ͼ���
		variance += diff * diff;

		// ���㷨������ǰһ���������ļн�
		if (i > 0) {
			Eigen::Vector3f prev_normal = cloud_with_normals->points[i - 1].getNormalVector3fMap();
			if (prev_normal.hasNaN()) {
				continue;
			}
			// ����н�
			float cos_angle = normal.dot(prev_normal) / (normal.norm() * prev_normal.norm());
			// ���Ƽн���[-1, 1]��Χ�ڣ�������ֵ���
			cos_angle = std::clamp(cos_angle, -1.0f, 1.0f);
			float angle = std::acos(cos_angle);  // ����н�
			angle_differences.push_back(angle);
		}
	}

	// �����������׼�
	variance /= cloud_with_normals->points.size();
	float normal_standard_deviation = std::sqrt(variance);

	// ���㷨�����нǵı�׼��
	float angle_variance = 0.0f;
	for (float angle : angle_differences) {
		angle_variance += angle * angle;
	}
	angle_variance /= angle_differences.size();
	float angle_standard_deviation = std::sqrt(angle_variance);

	// ����������ı�׼��ͷ������Ƕȵı�׼��
	std::cout << "Normal Vector Standard Deviation: " << normal_standard_deviation << std::endl;
	std::cout << "Normal Vector Angle Standard Deviation: " << angle_standard_deviation << " radians" << std::endl;
}

int main()
{
	
	//----------------------����˼��ɨ����Ƶķ�����һ����---------------------
	std::string filePath = "E:\\Repository\\Github\\LearningC-\\resource\\isolate\\1.txt";
	std::vector<Eigen::Vector3d> vNoramls;
	ReadFile(vNoramls, filePath);
	CalcNormalVariance(vNoramls);
	
	//----------------------��ȡ����---------------------
	pcl::PointCloud<PointT>::Ptr pCloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT>("2.pcd", *pCloud) == -1)   //statue2.pcd
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}

	//----------------------��������---------------------
	// �²�������ѡ���裬���ټ�������
	//pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::VoxelGrid<PointT> voxel_grid;
	//voxel_grid.setInputCloud(pCloud);
	//voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);  // �������صĳߴ�
	//voxel_grid.filter(*cloud_filtered);
	//pcl::PointCloud<PointT>::Ptr pSmoothedCloud(new pcl::PointCloud<PointT>());
	pcl::PointCloud<pcl::PointNormal>::Ptr pSmoothedCloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr pNormalCloud(new pcl::PointCloud<pcl::PointNormal>());
	//----------------------����ִ��--------------------
	auto startOp = std::chrono::high_resolution_clock::now();
	//MovingLeastSquares(pSmoothedCloud, pCloud);
	NeighborBasedNormal(pSmoothedCloud, pNormalCloud, pCloud);
	auto endOp = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp = endOp - startOp;
	std::cout << "����ƽ���㷨��ʱ: " << elapsedOp.count() << " seconds" << std::endl;

	// �жϷ�������һ����
	ComputeNormalVarianceAndAngle(pNormalCloud);
	ComputeNormalVarianceAndAngle(pSmoothedCloud);
	//float rms_error = 0.0;
	//for (size_t i = 0; i < pCloud->points.size(); ++i) {
	//	Eigen::Vector3f normal1 = pNormalCloud->points[i].getNormalVector3fMap();
	//	Eigen::Vector3f normal2 = pSmoothedCloud->points[i].getNormalVector3fMap();
	//	rms_error += std::pow(normal1.dot(normal2), 2);  // RMS of dot product
	//}
	//rms_error = std::sqrt(rms_error / pCloud->points.size());
	//std::cout << "RMS Error: " << rms_error << std::endl;

	//----------------------��ʾ���---------------------
#if ENABLE_DISPLAY
	cout << "ԭʼ���Ƶ���" << pCloud->points.size() << std::endl;
	cout << "ƽ������Ƶ���" << pSmoothedCloud->points.size() << std::endl;
	//VisualizedCloud(pSmoothedCloud, pCloud);
	pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
	GreedyTriangle(*triangles, pCloud, pNormalCloud);
	GreedyTriangle(*triangles, pCloud, pSmoothedCloud);
#endif
}