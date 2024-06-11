
// CloudCompare
#include <CCCoreLib/PointCloudTpl.h>
#include <CCCoreLib/GenericIndexedCloudPersist.h>
#include <CCCoreLib/CCGeom.h>
#include <CCCoreLib/GeometricalAnalysisTools.h>

// 标准文件
#include <string>
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

using PointCloud = CCCoreLib::GenericIndexedCloudPersist;

int main() {
	// ****************************获取数据******************************
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
	std::string fnameS = R"(bunny.pcd)";
	//支持pcd与ply两种格式
	if (fnameS.substr(fnameS.find_last_of('.') + 1) == "pcd") {
		pcl::io::loadPCDFile(fnameS, *pc);
	}
	else if (fnameS.substr(fnameS.find_last_of('.') + 1) == "ply") {
		pcl::io::loadPLYFile(fnameS, *pc);
	}

	// ****************************转换数据******************************
	CCCoreLib::PointCloudTpl<PointCloud> ccCloud;
	for (int i = 0; i < pc->points.size(); i++) {
		ccCloud.addPoint(CCVector3(pc->points[i].x,
			pc->points[i].y, pc->points[i].z));
	}

	std::cout << "点云中的点数量：" << ccCloud.size() << std::endl;

	// ****************************八叉树******************************
	float sphereRadius = 10.0f;

	std::shared_ptr<CCCoreLib::DgmOctree> octree(new CCCoreLib::DgmOctree(&ccCloud));
	if (octree->build() < 1)
	{
		std::cout << "八叉树失败！" << std::endl;
		return -1;
	}
	//找到执行计算的最佳八叉树级别
	unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(sphereRadius);
	CCCoreLib::DgmOctree::NeighboursSet neighbours;
	octree->getPointsInSphericalNeighbourhood(*ccCloud.getPoint(0), sphereRadius, neighbours, level);

	//提取出的邻域点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(neighbours.size());
	for (int i = 0; i < neighbours.size(); ++i)
	{
		const CCVector3* pt = neighbours[i].point;
		cloud->at(i).x = pt->x;
		cloud->at(i).y = pt->y;
		cloud->at(i).z = pt->z;
	}

	// ****************************保存数据******************************
	pcl::PLYWriter writer;
	writer.write("result.ply", *cloud, false);
	std::cout << "保存成功！" << std::endl;

	system("pause");
	return 0;
}

