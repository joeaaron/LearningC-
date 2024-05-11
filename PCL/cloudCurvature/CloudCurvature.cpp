#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <ctime>//c++�����ʱͷ�ļ�
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>

using namespace std;
clock_t start_time, end_time;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

//������Ƶ�n��������ʵ㡪����������
void MaxCurvaturePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int n, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

//���ʼ���ṹ��
typedef struct PCURVATURE {
	//POINT3F cPoint;
	int index;
	float curvature;
}PCURVATURE;

int main()
{
	//��ȡ��������
	if (pcl::io::loadPCDFile("bunny.pcd", *cloud))
	{
		std::cerr << "ERROR: Cannot open file " << std::endl;
		return 0;
	}

	//pcl::io::savePLYFile("test111.ply", *cloud);

	start_time = clock();//����ʼ��ʱ
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_MaxCurvature(new pcl::PointCloud<pcl::PointXYZ>);
	int nPointsNum = 0.2 * cloud->size();

	MaxCurvaturePoints(cloud, nPointsNum, cloud_MaxCurvature);//���Ƶ�5000��������ʵ�
	end_time = clock();//���������ʱ
	double endtime = (double)(end_time - start_time) / CLOCKS_PER_SEC;
	cout << "Total time:" << endtime << "s" << endl;//sΪ��λ
	cout << "Total time:" << endtime * 1000 << "ms" << endl;//msΪ��λ

	//���ӻ�
	pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
	viewer.initCameraParameters();
	viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_MaxCurvature, 255, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud_MaxCurvature, color, "cloud_MaxCurvature");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}

//������Ƶ�n��������ʵ㡪������ʵ��
void MaxCurvaturePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int nNum, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	//���Ʒ��߹���
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud_in);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normals_curvatures(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree_normals_curvatures);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	//ne.setRadiusSearch(0.03);
	ne.setKSearch(30);
	ne.compute(*cloud_normals);//���㷨��

	//�����������
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pc.setInputCloud(cloud_in);
	pc.setInputNormals(cloud_normals);
	pc.setSearchMethod(tree_normals_curvatures);
	pc.setRadiusSearch(0.02);//���������뾶������Ϊ0.01
	//pc.setKSearch(5);
	pc.compute(*cloud_curvatures);

	//������Ƶĸ�˹���ʣ���ȡ���ʼ�
	std::vector<PCURVATURE> allCurvates;
	pcl::PointXYZ tempPoint;
	float curvature = 0.0;
	PCURVATURE pv;
	tempPoint.x = tempPoint.y = tempPoint.z = 0.0;
	for (int i = 0; i < cloud_curvatures->size(); i++) {
		//ƽ������
		curvature = ((*cloud_curvatures)[i].pc1 + (*cloud_curvatures)[i].pc2) / 2;
		//��˹����
		//curvature = (*cloud_curvatures)[i].pc1 * (*cloud_curvatures)[i].pc2;
		//pv.cPoint = tempPoint;
		pv.index = i;
		pv.curvature = curvature;
		allCurvates.insert(allCurvates.end(), pv);
	}
	//��ȡ��������ǰn����
	PCURVATURE temp;
	int maxIndex = 0;
	int count = 0;
	for (int i = 0; i < allCurvates.size(); i++) {
		float maxCurvature = -99999;
		for (int j = i + 1; j < allCurvates.size(); j++) {
			if (maxCurvature < allCurvates[j].curvature) {
				maxCurvature = allCurvates[j].curvature;
				maxIndex = j;
			}
		}
		if (maxCurvature > allCurvates[i].curvature) {
			temp = allCurvates[maxIndex];
			allCurvates[maxIndex] = allCurvates[i];
			allCurvates[i] = temp;
			count++;
		}
		if (count > nNum) {
			break;
		}
	}
	//��ȡ������ʵ�
	for (int i = 0; i < nNum; i++)
	{
		int indexP = allCurvates[i].index;
		cloud_out->push_back(cloud_in->points[indexP]);
	}
}
