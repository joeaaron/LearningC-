
// CloudCompare
#include <CCCoreLib/PointCloudTpl.h>
#include <CCCoreLib/GenericIndexedCloudPersist.h>
#include <CCCoreLib/CCGeom.h>
#include <CCCoreLib/GeometricalAnalysisTools.h>

// ��׼�ļ�
#include <string>
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

using PointCloud = CCCoreLib::GenericIndexedCloudPersist;

int main() {
	// ****************************��ȡ����******************************
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
	std::string fnameS = R"(bunny.pcd)";
	//֧��pcd��ply���ָ�ʽ
	if (fnameS.substr(fnameS.find_last_of('.') + 1) == "pcd") {
		pcl::io::loadPCDFile(fnameS, *pc);
	}
	else if (fnameS.substr(fnameS.find_last_of('.') + 1) == "ply") {
		pcl::io::loadPLYFile(fnameS, *pc);
	}

	// ****************************ת������******************************
	CCCoreLib::PointCloudTpl<PointCloud> ccCloud;
	for (int i = 0; i < pc->points.size(); i++) {
		ccCloud.addPoint(CCVector3(pc->points[i].x,
			pc->points[i].y, pc->points[i].z));
	}

	std::cout << "�����еĵ�������" << ccCloud.size() << std::endl;

	// ****************************�˲���******************************
	float sphereRadius = 10.0f;

	std::shared_ptr<CCCoreLib::DgmOctree> octree(new CCCoreLib::DgmOctree(&ccCloud));
	if (octree->build() < 1)
	{
		std::cout << "�˲���ʧ�ܣ�" << std::endl;
		return -1;
	}
	//�ҵ�ִ�м������Ѱ˲�������
	unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(sphereRadius);
	CCCoreLib::DgmOctree::NeighboursSet neighbours;
	octree->getPointsInSphericalNeighbourhood(*ccCloud.getPoint(0), sphereRadius, neighbours, level);

	//��ȡ���������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(neighbours.size());
	for (int i = 0; i < neighbours.size(); ++i)
	{
		const CCVector3* pt = neighbours[i].point;
		cloud->at(i).x = pt->x;
		cloud->at(i).y = pt->y;
		cloud->at(i).z = pt->z;
	}

	// ****************************��������******************************
	pcl::PLYWriter writer;
	writer.write("result.ply", *cloud, false);
	std::cout << "����ɹ���" << std::endl;

	system("pause");
	return 0;
}

