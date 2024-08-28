
// CloudCompare
#include <CCCoreLib/PointCloudTpl.h>
#include <CCCoreLib/GenericIndexedCloudPersist.h>
#include <CCCoreLib/CCGeom.h>
#include <CCCoreLib/GeometricalAnalysisTools.h>
#include <CCCoreLib/Polyline.h>
#include <CCCoreLib/PointProjectionTools.h>
#include <CCCoreLib/CCTypes.h>

#ifdef CC_CORE_LIB_USES_TBB
#include <tbb/parallel_for.h>
#endif

// 标准文件
#include <string>
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/segmentation/extract_clusters.h>

using PointCloud = CCCoreLib::GenericIndexedCloudPersist;
using CCPolyline = CCCoreLib::Polyline;
using Vertex2D = CCCoreLib::PointProjectionTools::IndexedCCVector2;
using Hull2D = std::list<Vertex2D*>;
using VertexIterator = std::list<Vertex2D*>::iterator;

//list of already used point to avoid hull's inner loops
enum HullPointFlags {
	POINT_NOT_USED = 0,
	POINT_USED = 1,
	POINT_IGNORED = 2,
	POINT_FROZEN = 3,
};

// Axis dir
enum class AxisDir
{
	Axis_X = 0,
	Axis_Y,
	Axis_Z
};

namespace
{
	struct Edge
	{
		Edge() : nearestPointIndex(0), nearestPointSquareDist(-1.0f) {}

		Edge(const VertexIterator& A, unsigned _nearestPointIndex, float _nearestPointSquareDist)
			: itA(A)
			, nearestPointIndex(_nearestPointIndex)
			, nearestPointSquareDist(_nearestPointSquareDist)
		{}

		//operator
		inline bool operator< (const Edge& e) const { return nearestPointSquareDist < e.nearestPointSquareDist; }

		VertexIterator itA;
		unsigned nearestPointIndex;
		float nearestPointSquareDist;
	};

	void SavePointsToFile(const std::vector<Eigen::Vector2d>& points, const std::string& filename)
	{
		std::ofstream outFile(filename);
		if (!outFile.is_open()) {
			std::cerr << "Error opening file: " << filename << std::endl;
			return;
		}

		// Write each point to the file
		for (const auto& point : points) {
			outFile << point[0] << " " << point[1] << "\n";
		}

		outFile.close();
		std::cout << "Points saved to " << filename << std::endl;
	}

	bool Compare(const Eigen::Vector2d& a, const Eigen::Vector2d& b) 
	{
		//if (a.x() != b.x()) return a.x() < b.x();
		//return a.y() < b.y();
		return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
	}

	double CrossProduct(const Eigen::Vector2d& o, const Eigen::Vector2d& a, const Eigen::Vector2d& b) 
	{
		return (a - o).x() * (b - o).y() - (a - o).y() * (b - o).x();
	}

	std::vector<Eigen::Vector2d> ConvexHull(std::vector<Eigen::Vector2d> points) 
	{
		// 1. 排序点
		std::sort(points.begin(), points.end(), Compare);

		// 2. 构建下半部分
		std::vector<Eigen::Vector2d> lower;
		for (const auto& p : points) {
			while (lower.size() >= 2 && CrossProduct(lower[lower.size() - 2], lower.back(), p) < 0)
			{
				lower.pop_back();
			}
			lower.push_back(p);
		}

		// 3. 构建上半部分
		std::vector<Eigen::Vector2d> upper;
		for (auto it = points.rbegin(); it != points.rend(); ++it) 
		{
			const auto& p = *it;
			while (upper.size() >= 2 && CrossProduct(upper[upper.size() - 2], upper.back(), p) < 0) 
			{
				upper.pop_back();
			}
			upper.push_back(p);
		}

		// 4. 合并结果（去除重复的端点）
		lower.pop_back();
		upper.pop_back();
		lower.insert(lower.end(), upper.begin(), upper.end());

		return lower;
	}
}

//! Finds the nearest (available) point to an edge
/** \return The nearest point distance (or -1 if no point was found!)
**/
static PointCoordinateType FindNearestCandidate(unsigned& minIndex,
	const VertexIterator& itA,
	const VertexIterator& itB,
	const std::vector<Vertex2D>& points,
	const std::vector<HullPointFlags>& pointFlags,
	PointCoordinateType minSquareEdgeLength,
	bool allowLongerChunks = false,
	double minCosAngle = -1.0)
{
	//look for the nearest point in the input set
	PointCoordinateType minDist2 = -1;
	const CCVector2 AB = **itB - **itA;
	const PointCoordinateType squareLengthAB = AB.norm2();
	const unsigned pointCount = static_cast<unsigned>(points.size());

#ifdef CC_CORE_LIB_USES_TBB
	tbb::parallel_for(static_cast<unsigned int>(0), pointCount, [&](unsigned int i) {
		const Vertex2D& P = points[i];
		if (pointFlags[P.index] != POINT_NOT_USED)
			return;

		//skip the edge vertices!
		if (P.index == (*itA)->index || P.index == (*itB)->index)
		{
			return;
		}

		//we only consider 'inner' points
		const CCVector2 AP = P - **itA;
		if (AB.x * AP.y - AB.y * AP.x < 0)
		{
			return;
		}

		//check the angle
		if (minCosAngle > -1.0)
		{
			const CCVector2 PB = **itB - P;
			const PointCoordinateType dotProd = AP.x * PB.x + AP.y * PB.y;
			const PointCoordinateType minDotProd = static_cast<PointCoordinateType>(minCosAngle * std::sqrt(AP.norm2() * PB.norm2()));
			if (dotProd < minDotProd)
			{
				return;
			}
		}

		const PointCoordinateType dot = AB.dot(AP); // = cos(PAB) * ||AP|| * ||AB||
		if (dot >= 0 && dot <= squareLengthAB)
		{
			const CCVector2 HP = AP - AB * (dot / squareLengthAB);
			const PointCoordinateType dist2 = HP.norm2();
			if (minDist2 < 0 || dist2 < minDist2)
			{
				//the 'nearest' point must also be a valid candidate
				//(i.e. at least one of the created edges is smaller than the original one
				//and we don't create too small edges!)
				const PointCoordinateType squareLengthAP = AP.norm2();
				const PointCoordinateType squareLengthBP = (P - **itB).norm2();
				if (squareLengthAP >= minSquareEdgeLength
					&& squareLengthBP >= minSquareEdgeLength
					&& (allowLongerChunks || (squareLengthAP < squareLengthAB || squareLengthBP < squareLengthAB))
					)
				{
					minDist2 = dist2;
					minIndex = i;
				}
			}
		}
		});
#else
	for (unsigned i = 0; i < pointCount; ++i)
	{
		const Vertex2D& P = points[i];
		if (pointFlags[P.index] != POINT_NOT_USED)
			continue;

		//skip the edge vertices!
		if (P.index == (*itA)->index || P.index == (*itB)->index)
		{
			continue;
		}

		//we only consider 'inner' points
		CCVector2 AP = P - **itA;
		if (AB.x * AP.y - AB.y * AP.x < 0)
		{
			continue;
		}

		//check the angle
		if (minCosAngle > -1.0)
		{
			CCVector2 PB = **itB - P;
			PointCoordinateType dotProd = AP.x * PB.x + AP.y * PB.y;
			PointCoordinateType minDotProd = static_cast<PointCoordinateType>(minCosAngle * std::sqrt(AP.norm2() * PB.norm2()));
			if (dotProd < minDotProd)
			{
				continue;
			}
		}

		PointCoordinateType dot = AB.dot(AP); // = cos(PAB) * ||AP|| * ||AB||
		if (dot >= 0 && dot <= squareLengthAB)
		{
			CCVector2 HP = AP - AB * (dot / squareLengthAB);
			PointCoordinateType dist2 = HP.norm2();
			if (minDist2 < 0 || dist2 < minDist2)
			{
				//the 'nearest' point must also be a valid candidate
				//(i.e. at least one of the created edges is smaller than the original one
				//and we don't create too small edges!)
				PointCoordinateType squareLengthAP = AP.norm2();
				PointCoordinateType squareLengthBP = (P - **itB).norm2();
				if (squareLengthAP >= minSquareEdgeLength
					&& squareLengthBP >= minSquareEdgeLength
					&& (allowLongerChunks || (squareLengthAP < squareLengthAB || squareLengthBP < squareLengthAB))
					)
				{
					minDist2 = dist2;
					minIndex = i;
				}
			}
		}
	}
#endif

	return (minDist2 < 0 ? minDist2 : minDist2 / squareLengthAB);
}

bool ExtractConcaveHull2D(std::vector<Vertex2D>& points,
	std::list<Vertex2D*>& hullPoints,
	PointCoordinateType maxSquareEdgeLength/*=0*/,
	double maxAngleDeg/*=0.0*/)
{
	//first compute the Convex hull
	if (!CCCoreLib::PointProjectionTools::extractConvexHull2D(points, hullPoints))
		return false;

	//std::vector<Eigen::Vector2d> v2DPoints;
	//for (auto vertex : hullPoints) 
	//{
	//	Eigen::Vector2d point(vertex->x, vertex->y);
	//	v2DPoints.push_back(point);
	//}

	//SavePointsToFile(v2DPoints, "Hull points.txt");
	
	if (hullPoints.size() < 2 || maxSquareEdgeLength < 0)
		return true;

	unsigned pointCount = static_cast<unsigned>(points.size());

	std::vector<HullPointFlags> pointFlags;
	try
	{
		pointFlags.resize(pointCount, POINT_NOT_USED);
	}
	catch (...)
	{
		//not enough memory
		return false;
	}

	double minCosAngle = maxAngleDeg <= 0 ? -1.0 : std::cos(maxAngleDeg * M_PI / 180.0);

	//hack: compute the theoretical 'minimal' edge length
	PointCoordinateType minSquareEdgeLength = 0;
	{
		CCVector2 minP;
		CCVector2 maxP;
		for (size_t i = 0; i < pointCount; ++i)
		{
			const Vertex2D& P = points[i];
			if (i)
			{
				minP.x = std::min(P.x, minP.x);
				minP.y = std::min(P.y, minP.y);
				maxP.x = std::max(P.x, maxP.x);
				maxP.y = std::max(P.y, maxP.y);
			}
			else
			{
				minP = maxP = P;
			}
		}

		minSquareEdgeLength = (maxP - minP).norm2() / static_cast<PointCoordinateType>(1.0e7); //10^-7 of the max bounding rectangle side
		minSquareEdgeLength = std::min(minSquareEdgeLength, maxSquareEdgeLength / 10);
		
		//we remove very small edges
		for (VertexIterator itA = hullPoints.begin(); itA != hullPoints.end(); ++itA)
		{
			VertexIterator itB = itA; ++itB;
			if (itB == hullPoints.end())
				itB = hullPoints.begin();
			if ((**itB - **itA).norm2() < minSquareEdgeLength)
			{
				pointFlags[(*itB)->index] = POINT_FROZEN;
				hullPoints.erase(itB);
			}
		}

		if (hullPoints.size() < 2)
		{
			//no more edges?!
			return false;
		}
	}

	unsigned step = 0;
	bool somethingHasChanged = true;
	while (somethingHasChanged)
	{
		try
		{
			somethingHasChanged = false;
			++step;

			//build the initial edge list & flag the convex hull points
			std::multiset<Edge> edges;
			//initial number of edges
			assert(hullPoints.size() >= 2);
			size_t initEdgeCount = hullPoints.size();
			VertexIterator itB = hullPoints.begin();
			for (size_t i = 0; i < initEdgeCount; ++i)
			{
				VertexIterator itA = itB; ++itB;
				if (itB == hullPoints.end())
					itB = hullPoints.begin();

				//we will only process the edges that are longer than the maximum specified length
				if ((**itB - **itA).norm2() > maxSquareEdgeLength)
				{
					unsigned nearestPointIndex = 0;
					PointCoordinateType minSquareDist = FindNearestCandidate(
						nearestPointIndex,
						itA,
						itB,
						points,
						pointFlags,
						minSquareEdgeLength,
						step > 1,
						minCosAngle);

					if (minSquareDist >= 0)
					{
						Edge e(itA, nearestPointIndex, minSquareDist);
						edges.insert(e);
					}
				}

				pointFlags[(*itA)->index] = POINT_USED;
			}
			//flag the last vertex as well for non closed envelopes!
			while (!edges.empty())
			{
				//current edge (AB)
				//this should be the edge with the nearest 'candidate'
				Edge e = *edges.begin();
				edges.erase(edges.begin());

				VertexIterator itA = e.itA;
				VertexIterator itB = itA; ++itB;
				if (itB == hullPoints.end())
				{
					itB = hullPoints.begin();
				}

				//nearest point
				const Vertex2D& P = points[e.nearestPointIndex];
				//assert(pointFlags[P.index] == POINT_NOT_USED); //we don't consider already used points!

				//last check: the new segments must not intersect with the actual hull!
				bool intersect = false;
				{
					for (VertexIterator itJ = hullPoints.begin(), itI = itJ++; itI != hullPoints.end(); ++itI, ++itJ)
					{
						if (itJ == hullPoints.end())
						{	
							itJ = hullPoints.begin();
						}

						if (((*itI)->index != (*itA)->index && (*itJ)->index != (*itA)->index && CCCoreLib::PointProjectionTools::segmentIntersect(**itI, **itJ, **itA, P))
							|| ((*itI)->index != (*itB)->index && (*itJ)->index != (*itB)->index && CCCoreLib::PointProjectionTools::segmentIntersect(**itI, **itJ, P, **itB)))
						{
							intersect = true;
							break;
						}
					}
				}
				if (!intersect)
				{
					//add point to concave hull
					VertexIterator itP = hullPoints.insert(itB == hullPoints.begin() ? hullPoints.end() : itB, &points[e.nearestPointIndex]);

					//we won't use P anymore!
					pointFlags[P.index] = POINT_USED;

					somethingHasChanged = true;
					
					//update all edges that were having 'P' as their nearest candidate as well
					if (!edges.empty())
					{
						std::vector<VertexIterator> removed;
						std::multiset<Edge>::const_iterator lastValidIt = edges.end();
						for (std::multiset<Edge>::const_iterator it = edges.begin(); it != edges.end(); ++it)
						{
							if ((*it).nearestPointIndex == e.nearestPointIndex)
							{
								//we'll have to put them back afterwards!
								removed.push_back((*it).itA);

								edges.erase(it);
								if (edges.empty())
									break;
								if (lastValidIt != edges.end())
									it = lastValidIt;
								else
									it = edges.begin();
							}
							else
							{
								lastValidIt = it;
							}
						}

						//update the removed edges info and put them back in the main list
						for (size_t i = 0; i < removed.size(); ++i)
						{
							VertexIterator itC = removed[i];
							VertexIterator itD = itC; ++itD;
							if (itD == hullPoints.end())
								itD = hullPoints.begin();

							unsigned nearestPointIndex = 0;
							PointCoordinateType minSquareDist = FindNearestCandidate(
								nearestPointIndex,
								itC,
								itD,
								points,
								pointFlags,
								minSquareEdgeLength,
								false,
								minCosAngle);

							if (minSquareDist >= 0)
							{
								Edge e(itC, nearestPointIndex, minSquareDist);
								edges.insert(e);
							}
						}
					}
					
					//we'll inspect the two new segments later (if necessary)
					if ((P - **itA).norm2() > maxSquareEdgeLength)
					{
						unsigned nearestPointIndex = 0;
						PointCoordinateType minSquareDist = FindNearestCandidate(
							nearestPointIndex,
							itA,
							itP,
							points,
							pointFlags,
							minSquareEdgeLength,
							false,
							minCosAngle);

						if (minSquareDist >= 0)
						{
							Edge e(itA, nearestPointIndex, minSquareDist);
							edges.insert(e);
						}
					}
					if ((**itB - P).norm2() > maxSquareEdgeLength)
					{
						unsigned nearestPointIndex = 0;
						PointCoordinateType minSquareDist = FindNearestCandidate(
							nearestPointIndex,
							itP,
							itB,
							points,
							pointFlags,
							minSquareEdgeLength,
							false,
							minCosAngle);

						if (minSquareDist >= 0)
						{
							Edge e(itP, nearestPointIndex, minSquareDist);
							edges.insert(e);
						}
					}
				}
			}
		}
		catch (...)
		{
			//not enough memory
			return false;
		}
	}

	return true;
}

// 计算相邻点之间的距离，并根据阈值判断
std::vector<bool> CheckDistances(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double threshold) 
{
	size_t numPoints = cloud->points.size();
	std::vector<bool> distancesValid(numPoints);

	if (numPoints < 2) {
		std::cerr << "Not enough points to compute distances." << std::endl;
		return distancesValid;
	}

#pragma omp parallel for
	for (int i = 0; i < numPoints; ++i) 
	{
		size_t nextIndex = (i + 1) % numPoints; // 最后一个点与第一个点相连
		const pcl::PointXYZ& pt1 = cloud->points[i];
		const pcl::PointXYZ& pt2 = cloud->points[nextIndex];

		double distance = std::sqrt(
			std::pow(pt1.x - pt2.x, 2) +
			std::pow(pt1.y - pt2.y, 2) +
			std::pow(pt1.z - pt2.z, 2)
		);

		// 判断距离是否在阈值范围内
		distancesValid[i] = (distance <= threshold);
	}

	return distancesValid;
}

CCPolyline* ExtractFlatEnvelope(PointCloud* points,
	PointCoordinateType maxEdgeLength,
	const PointCoordinateType* preferredNormDim/*=nullptr*/,
	const PointCoordinateType* preferredUpDir/*=nullptr*/)
{
	assert(points);

	if (!points)
		return nullptr;

	unsigned ptsCount = points->size();

	if (ptsCount < 3)
		return nullptr;

	CCCoreLib::Neighbourhood Yk(points);

	//local base
	CCVector3 O;
	CCVector3 X;
	CCVector3 Y;

	CCCoreLib::Neighbourhood::InputVectorsUsage vectorsUsage = CCCoreLib::Neighbourhood::None;

	//we project the input points on a plane
	std::vector<Vertex2D> points2D;
	PointCoordinateType* planeEq = nullptr;

	if (preferredUpDir != nullptr)
	{
		Y = CCVector3(preferredUpDir);
		vectorsUsage = CCCoreLib::Neighbourhood::UseYAsUpDir;
	}

	//if the user has specified a default direction, we'll use it as 'projecting plane'
	PointCoordinateType preferredPlaneEq[4] = { 0, 0, 1, 0 };

	if (preferredNormDim != nullptr)
	{
		const CCVector3* G = points->getPoint(0); //any point through which the plane passes is ok
		//const CCVector3* G = new CCVector3(1.321, 0, -12.500);  // 通过平面中心点
		//const CCVector3* G = new CCVector3(-0.001, 0, -17.756); 
		preferredPlaneEq[0] = preferredNormDim[0];
		preferredPlaneEq[1] = preferredNormDim[1];
		preferredPlaneEq[2] = preferredNormDim[2];
		CCVector3::vnormalize(preferredPlaneEq);
		preferredPlaneEq[3] = CCVector3::vdot(G->u, preferredPlaneEq);
		planeEq = preferredPlaneEq;

		if (preferredUpDir != nullptr)
		{
			O = *G;
			//Y = CCVector3(preferredUpDir); //already done above
			X = Y.cross(CCVector3(preferredNormDim));
			vectorsUsage = CCCoreLib::Neighbourhood::UseOXYasBase;
		}
	}

	if (!Yk.projectPointsOn2DPlane<Vertex2D>(points2D, planeEq, &O, &X, &Y, vectorsUsage))
	{
		return nullptr;
	}

	// 调试用
	std::vector<Eigen::Vector2d> v2DPoints;
	for (int i = 0; i < points2D.size(); ++i)
	{
		v2DPoints.emplace_back(Eigen::Vector2d(points2D[i].x, points2D[i].y));
	}
	SavePointsToFile(v2DPoints, "Projected points.txt");

	std::vector<Eigen::Vector2d> vHullPoints = ConvexHull(v2DPoints);
	SavePointsToFile(vHullPoints, "Hulled points.txt");

	//update the points indexes (not done by Neighbourhood::projectPointsOn2DPlane)
	{
		for (unsigned i = 0; i < ptsCount; ++i)
		{
			points2D[i].index = i;
		}
	}

	//try to get the points on the convex/concave hull to build the envelope and the polygon
	Hull2D hullPoints;
	if (!ExtractConcaveHull2D(points2D,
		hullPoints,
		maxEdgeLength * maxEdgeLength, 0))
	{
		return nullptr;
	}

	unsigned hullPtsCount = static_cast<unsigned>(hullPoints.size());

	//create vertices
	CCCoreLib::PointCloudTpl<PointCloud> envelopeVertices;
	{
		//projection on the LS plane (in 3D)
		for (Hull2D::const_iterator it = hullPoints.begin(); it != hullPoints.end(); ++it)
		{
			envelopeVertices.addPoint(O + X * (*it)->x + Y * (*it)->y);
		}
	}
	CCCoreLib::GenericIndexedCloudPersist* envVertices = static_cast<CCCoreLib::GenericIndexedCloudPersist*>(&envelopeVertices);
	//we create the corresponding (3D) polyline
	CCPolyline* envelopePolyline = new CCPolyline(envVertices);
	if (envelopePolyline->reserve(hullPtsCount))
	{
		envelopePolyline->addPointIndex(0, hullPtsCount);
		envelopePolyline->setClosed(true);
	}
	else
	{
		delete envelopePolyline;
		envelopePolyline = nullptr;
	}

	// ****************************断面显示******************************
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 遍历 polyline 的顶点，并将其添加到 PCL 点云对象中
	int nSize = envVertices->size();
	cloud->points.reserve(nSize);
	for (unsigned int i = 0; i < nSize; ++i)
	{
		CCVector3 point;
		envVertices->getPoint(i, point);
		cloud->points.push_back(pcl::PointXYZ(point.x, point.y, point.z));
	}


	// 遍历点云并顺序连接相邻的点
	double dSamplingStep = 0.5;
	// 创建可视化器
	pcl::visualization::PCLVisualizer viewer("Line Viewer");
	for (size_t i = 0; i < cloud->points.size() - 1; ++i)
	{
		std::string line_id = "line_" + std::to_string(i);
		double dis = pcl::euclideanDistance(cloud->points[i], cloud->points[i + 1]);
		if (dis > dSamplingStep * 9)
			continue;

		viewer.addLine(cloud->points[i], cloud->points[i + 1], line_id);
	}
	// 连接最后一个点到第一个点，形成闭环
	//viewer.addLine(sortedPoints->points.back(), sortedPoints->points.front(), "line_close");
	viewer.resetCamera();

	// 运行可视化器
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	//// 使用 PCL 可视化工具显示点云
	//pcl::visualization::CloudViewer viewer("Polyline Viewer");
	//viewer.showCloud(cloud);

	//// 等待直到视图关闭
	//while (!viewer.wasStopped()) {}

	// 创建KD-Tree对象用于搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	// 欧几里德聚类提取器
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(5);		// 设置近邻搜索的搜索半径
	ec.setMinClusterSize(20);		// 设置一个聚类需要的最少点数目
	ec.setMaxClusterSize(100000);		// 设置一个聚类需要的最大点数目
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	/////////////////////////显示1///////////////////////
	std::vector<pcl::PointCloud<pcl::PointXYZ>> vCloud;
	for (const auto& indices : cluster_indices) 
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (const auto& index : indices.indices)
			cloud_cluster->points.push_back(cloud->points[index]);

		vCloud.emplace_back(*cloud_cluster);

		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// 使用 PCL 可视化工具显示点云
		pcl::visualization::CloudViewer viewer("Polyline Viewer");
		viewer.showCloud(cloud_cluster);

		// 等待直到视图关闭
		while (!viewer.wasStopped()) {}
	}
	
	
	 // Create a new point cloud for colored clusters
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	colored_cloud->header = cloud->header;
	colored_cloud->width = cloud->width;
	colored_cloud->height = cloud->height;
	colored_cloud->is_dense = cloud->is_dense;

	int cluster_id = 0;

	for (const auto& indices : cluster_indices) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		cluster_cloud->width = indices.indices.size();
		cluster_cloud->height = 1;
		cluster_cloud->points.resize(cluster_cloud->width * cluster_cloud->height);

		// Assign a unique color to this cluster
		pcl::PointXYZRGB color;
		color.r = (rand() % 256);
		color.g = (rand() % 256);
		color.b = (rand() % 256);

		for (size_t i = 0; i < indices.indices.size(); ++i) {
			pcl::PointXYZRGB& pt = cluster_cloud->points[i];
			pt.x = cloud->points[indices.indices[i]].x;
			pt.y = cloud->points[indices.indices[i]].y;
			pt.z = cloud->points[indices.indices[i]].z;
			pt.r = color.r;
			pt.g = color.g;
			pt.b = color.b;
		}

		*colored_cloud += *cluster_cloud;
	}

	// Visualize the colored point cloud
	pcl::visualization::PCLVisualizer viewer2("Cluster Viewer");
	viewer2.addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");

	while (!viewer2.wasStopped()) {
		viewer2.spinOnce(100);
	}

	//// 计算相邻点距离
	//double threshold = 1.5;				// 设置采样间距
	//std::vector<bool> validDistances = CheckDistances(cloud, threshold);

	//// 输出结果
	//for (size_t i = 0; i < validDistances.size(); ++i) 
	//{
	//	std::cout << "Distance between point " << i << " and " << ((i + 1) % cloud->points.size()) << " is ";
	//	if (validDistances[i]) 
	//	{
	//		std::cout << "within threshold" << std::endl;
	//	}
	//	else {
	//		std::cout << "out of threshold" << std::endl;
	//	}
	//}
	
	return envelopePolyline;
}

inline Eigen::Vector4d CalcPlane(Eigen::Vector3d center, Eigen::Vector3d normal)
{
	Eigen::Vector4d planeEquation;

	// 计算平面方程中的 d
	double d = normal.dot(center);
	planeEquation << normal, -d;

	return planeEquation;
}

void GetSlice(pcl::PointCloud<pcl::PointXYZ>::Ptr& sliceCloud, 
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
	const Eigen::Vector4d& n,
	double delta)
{
	//double delta = 2;			// 设置切片的0.5倍厚度,厚度和点云密度相关
	std::vector<int> pointIdx;

	for (int i = 0; i < cloud->size(); ++i)
	{
		double wr = n[0] * (*cloud)[i].x + n[1] * (*cloud)[i].y + n[2] * (*cloud)[i].z + n[3] - delta;
		double wl = n[0] * (*cloud)[i].x + n[1] * (*cloud)[i].y + n[2] * (*cloud)[i].z + n[3] + delta;
		if (wr * wl <= 0)
		{
			pointIdx.emplace_back(i);
		}
	}

	pcl::copyPointCloud(*cloud, pointIdx, *sliceCloud);
	pcl::PCDWriter writer;
	writer.write("sliceCloud.pcd", *sliceCloud, false);
	// ****************************包围盒内点云显示******************************
	pcl::visualization::CloudViewer viewer("CropBox Viewer");
	viewer.showCloud(sliceCloud);
	
	// 等待直到视图关闭
	while (!viewer.wasStopped()) {}
}

// 根据法向量获取旋转矩阵
Eigen::Matrix3d GetRotationMatrix(const Eigen::Vector3d& normal)
{
	// 创建一个单位向量
	Eigen::Vector3d up(0.0, 0.0, 1.0);

	// 计算旋转轴
	Eigen::Vector3d axis = up.cross(normal);
	axis.normalize();

	// 计算旋转角度
	double angle = acos(up.dot(normal) / (up.norm() * normal.norm()));

	// 使用旋转轴和角度创建旋转矩阵
	Eigen::AngleAxisd angleAxis(angle, axis);
	Eigen::Matrix3d rotationMatrix = angleAxis.toRotationMatrix();

	return rotationMatrix;
}

// 根据平面方程获取变换矩阵
Eigen::Matrix4d GetTransformMatrix(Eigen::Vector4d plane) 
{
	// 创建法向量
	Eigen::Vector3d normal = plane.head<3>();
	normal.normalize();

	// 获取旋转矩阵
	Eigen::Matrix3d rotationMatrix = GetRotationMatrix(normal);

	// 创建平移矩阵
	Eigen::Vector3d translation;// (0, 0, -D / C);

	// 组合平移和旋转为4x4变换矩阵
	Eigen::Matrix4d transformMatrix = Eigen::Matrix4d::Identity();
	transformMatrix.block<3, 3>(0, 0) = rotationMatrix;
	transformMatrix.block<3, 1>(0, 3) = translation;

	return transformMatrix;
}

double CalcBoxDis(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);

	// 计算对角线长度
	double dLength = std::sqrt(std::pow(maxPt.x - minPt.x, 2) +
		std::pow(maxPt.y - minPt.y, 2) +
		std::pow(maxPt.z - minPt.z, 2));

	return dLength;
}

bool Mesh2CloudPCL(pcl::PointCloud<pcl::PointXYZ>& cloudOut,
	const pcl::PolygonMesh& mesh)
{
	pcl::fromPCLPointCloud2(mesh.cloud, cloudOut);
	return true;
}

int main() 
{
	// ****************************获取数据******************************
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PolygonMesh mesh;
	std::string fnameS = R"(pmt.pcd)";   //pmt.pcd | bunny.pcd | Testcase01.pcd | test.stl | Testcase02.pcd
	//支持pcd与ply两种格式
	if (fnameS.substr(fnameS.find_last_of('.') + 1) == "pcd") {
		pcl::io::loadPCDFile(fnameS, *pc);
	}
	else if (fnameS.substr(fnameS.find_last_of('.') + 1) == "ply") {
		pcl::io::loadPLYFile(fnameS, *pc);
	}
	else if (fnameS.substr(fnameS.find_last_of('.') + 1) == "stl") {
		pcl::io::loadPolygonFileSTL(fnameS, mesh);
	}

	// 创建KD-Tree对象用于搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(pc);
	
	// 计算点云密度
	double dAvgDis = 0.0;
	int nNum = 0;
	for (int i = 0; i < pc->size(); ++i)
	{
		std::vector<int> indiceId;
		std::vector<float> disSquare;

		if (tree->nearestKSearch(pc->points[i], 2, indiceId, disSquare) > 0)
		{
			dAvgDis += sqrt(disSquare[1]);
			nNum++;
		}
	}
	dAvgDis /= nNum;

	//Mesh2CloudPCL(*pc, mesh);
	// ****************************获取包围盒内的点云******************************
	//Eigen::Vector3d center(-4.349, 0, -12.500);   //(88.328, 0.000, 4.938) | (-4.349, 0, -12.500);  testcase1 | PMT
	//Eigen::Vector3d center(0, 0, -12.500);
	// X轴
	Eigen::Vector3d center(1.321, 0, -12.500);
	Eigen::Vector3d normal(1, 0, 0);

	// Z轴
	//Eigen::Vector3d center(-0.001, 0, -17.7557755/* -13.305*/);
	//Eigen::Vector3d normal(0, 0, 1);

	//Eigen::Vector3d center(-0.001, 0, -18.660); 
	//Eigen::Vector3d center(377.767, -409.888, 395.581);
	//Eigen::Vector3d normal(0, 0, 1);

	Eigen::Vector4d plane = CalcPlane(center, normal);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sliceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	GetSlice(sliceCloud, pc, plane, 2);

	// 调试用
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("SliceCloudZ.pcd", *sliceCloud);

	// 根据包围盒对角线距离设置maxEdgeLength
	PointCoordinateType maxEdgeLength = CalcBoxDis(sliceCloud) / 100.0;  

	// ****************************转换数据******************************
	CCCoreLib::PointCloudTpl<PointCloud> ccCloud;
	for (int i = 0; i < sliceCloud->points.size(); i++) 
	{
		ccCloud.addPoint(CCVector3(sliceCloud->points[i].x,
			sliceCloud->points[i].y, sliceCloud->points[i].z));
	}

	std::cout << "点云中的点数量：" << ccCloud.size() << std::endl;

	// ****************************切片算法******************************
	PointCloud* points = static_cast<PointCloud*>(&ccCloud);
	AxisDir axis = AxisDir::Axis_Z;

	PointCoordinateType* preferredNormDir = nullptr;
	PointCoordinateType* preferredUpDir = nullptr;
	if (axis == AxisDir::Axis_Z)
	{
		preferredNormDir = new PointCoordinateType[3]{ 0.0, 0.0, 1.0 };
		preferredUpDir = new PointCoordinateType[3]{ 0.0, -1.0, 0.0 };
	}
	else if (axis == AxisDir::Axis_Y)
	{
		preferredNormDir = new PointCoordinateType[3]{ 0.0, 1.0, 0.0 };
		preferredUpDir = new PointCoordinateType[3]{ 0.0, 0.0, 1.0 };
	}
	else
	{
		preferredNormDir = new PointCoordinateType[3] { 1.0, 0.0, 0.0 };
		preferredUpDir = new PointCoordinateType[3]{ 0.0, 0.0, 1.0 };
	}

	CCPolyline* polyLine = ExtractFlatEnvelope(points, maxEdgeLength, preferredNormDir, preferredUpDir);

	//// ****************************八叉树******************************
	//float sphereRadius = 10.0f;

	//std::shared_ptr<CCCoreLib::DgmOctree> octree(new CCCoreLib::DgmOctree(&ccCloud));
	//if (octree->build() < 1)
	//{
	//	std::cout << "八叉树失败！" << std::endl;
	//	return -1;
	//}
	////找到执行计算的最佳八叉树级别
	//unsigned char level = octree->findBestLevelForAGivenNeighbourhoodSizeExtraction(sphereRadius);
	//CCCoreLib::DgmOctree::NeighboursSet neighbours;
	//octree->getPointsInSphericalNeighbourhood(*ccCloud.getPoint(0), sphereRadius, neighbours, level);

	////提取出的邻域点
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//cloud->resize(neighbours.size());
	//for (int i = 0; i < neighbours.size(); ++i)
	//{
	//	const CCVector3* pt = neighbours[i].point;
	//	cloud->at(i).x = pt->x;
	//	cloud->at(i).y = pt->y;
	//	cloud->at(i).z = pt->z;
	//}

	//// ****************************保存数据******************************
	//pcl::PLYWriter writer;
	//writer.write("result.ply", *cloud, false);
	//std::cout << "保存成功！" << std::endl;

	system("pause");
	return 0;
}

