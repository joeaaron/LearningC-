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
#include <algorithm>

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

#define ENABLE_DISPLAY 1					// 定义一个宏，用于控制显示状态

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

double CalcCloudDensity(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// 创建KD-Tree对象用于搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	// 计算点云密度
	double dAvgDis = 0.0;
	int nNum = 0;
	for (int i = 0; i < cloud->size(); ++i)
	{
		std::vector<int> indiceId;
		std::vector<float> disSquare;

		if (tree->nearestKSearch(cloud->points[i], 2, indiceId, disSquare) > 0)
		{
			dAvgDis += sqrt(disSquare[1]);
			nNum++;
		}
	}
	dAvgDis /= nNum;

	return dAvgDis;
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
std::vector<bool> CheckDistances(double& dAvg,
	double& dStd, double& dMax, double& dMin,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
	double threshold) 
{
	size_t numPoints = cloud->points.size();
	std::vector<bool> distancesValid(numPoints);

	if (numPoints < 2) {
		std::cerr << "Not enough points to compute distances." << std::endl;
		return distancesValid;
	}

	std::vector<double> vDis;
//#pragma omp parallel for
	for (int i = 0; i < numPoints; ++i) 
	{
		size_t nextIndex = (i + 1) % numPoints; // 最后一个点与第一个点相连
		const pcl::PointXYZ& pt1 = cloud->points[i];
		const pcl::PointXYZ& pt2 = cloud->points[nextIndex];
		auto distance = pcl::euclideanDistance(pt1, pt2);
		/*double distance = std::sqrt(
			std::pow(pt1.x - pt2.x, 2) +
			std::pow(pt1.y - pt2.y, 2) +
			std::pow(pt1.z - pt2.z, 2)
		);*/
		vDis.emplace_back(distance);
		// 判断距离是否在阈值范围内
		distancesValid[i] = (distance <= threshold);
	}
	dAvg = std::accumulate(vDis.begin(), vDis.end(), 0.0) / vDis.size();
	double dSqu = std::inner_product(vDis.begin(), vDis.end(), vDis.begin(), 0.0f);
	dStd = std::sqrt(dSqu / vDis.size() - dAvg * dAvg);

	dMax = *std::max_element(vDis.begin(), vDis.end());
	dMin = *std::min_element(vDis.begin(), vDis.end());

	/*double variance = 0.0;
	for (double value : vDis) {
		variance += (value - dAvg) * (value - dAvg);
	}
	variance /= vDis.size();
	dStd = std::sqrt(variance);*/

	return distancesValid;
}

// 计算点云的平均距离和标准差
void ComputeDistanceStats(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	double& average_distance, double& std_dev) 
{
	std::vector<double> distances;
	pcl::search::KdTree<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->points.size(); ++i) 
	{
		std::vector<int> point_indices;
		std::vector<float> point_distances;
		kdtree.nearestKSearch(cloud->points[i], 2, point_indices, point_distances);
		if (point_distances.size() > 1) 
		{
			distances.push_back(point_distances[1]); // 距离第一个最近邻的点
		}
	}

	// 计算平均距离
	float sum = std::accumulate(distances.begin(), distances.end(), 0.0f);
	average_distance = sum / distances.size();

	// 计算标准差
	float sq_sum = std::inner_product(distances.begin(), distances.end(), distances.begin(), 0.0f);
	std_dev = std::sqrt(sq_sum / distances.size() - average_distance * average_distance);
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
		//const CCVector3* G = new CCVector3(1.947, 3.896, -6.655);
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
		maxEdgeLength*maxEdgeLength, 0))
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
	cloud->width = cloud->points.size();
	cloud->height = 1;

	// 保存切片点云
	//pcl::PCDWriter writer;
	//writer.write<pcl::PointXYZ>("ResCloud.pcd", *cloud);

	/*pcl::PointXYZ min_pt, max_pt;
	pcl::getMinMax3D(*cloud, min_pt, max_pt);*/

	// 计算包围盒的中心点
	//pcl::PointXYZ center;
	//center.x = (min_pt.x + max_pt.x) / 2.0;
	//center.y = (min_pt.y + max_pt.y) / 2.0;
	//center.z = (min_pt.z + max_pt.z) / 2.0;

	//// 计算包围盒的宽度、高度和深度
	//double width = max_pt.x - min_pt.x;
	//double height = max_pt.y - min_pt.y;
	//double depth = max_pt.z - min_pt.z;

	//// 输出结果
	//std::cout << "Center point: (" << center.x << ", " << center.y << ", " << center.z << ")" << std::endl;
	//std::cout << "Width: " << width << std::endl;
	//std::cout << "Height: " << height << std::endl;
	//std::cout << "Depth: " << depth << std::endl;

	// 多段线
//#if ENABLE_DISPLAY
//	// 遍历点云并顺序连接相邻的点
//	double dSamplingStep = 0.5;
//	// 创建可视化器
//	pcl::visualization::PCLVisualizer viewer("Line Viewer");
//	for (size_t i = 0; i < cloud->points.size() - 1; ++i)
//	{
//		std::string line_id = "line_" + std::to_string(i);
//		double dis = pcl::euclideanDistance(cloud->points[i], cloud->points[i + 1]);
//		if (dis > dSamplingStep * 4)
//			continue;
//
//		viewer.addLine(cloud->points[i], cloud->points[i + 1], line_id);
//	}
//	viewer.resetCamera();
//
//	// 等待直到视图关闭
//	while (!viewer.wasStopped()) 
//	{
//		viewer.spinOnce(100);
//	}
//#endif

	double threshold = 1.5;				// 设置采样间距
	double dAvg = 0.0;					// 平均距离
	double dStd = 0.0;					// 标准差距离
	double dMax = 0.0;					// 最大距离
	double dMin = 0.0;					// 最小距离
	std::vector<bool> validDistances = CheckDistances(dAvg, dStd, dMax, dMin, cloud, threshold);
	//ComputeDistanceStats(cloud, dAvg, dStd);
	// 创建KD-Tree对象用于搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	// 欧几里德聚类提取器
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(dAvg+dStd);			// 设置近邻搜索的搜索半径
	ec.setMinClusterSize(3);			// 设置一个聚类需要的最少点数目
	ec.setMaxClusterSize(1000);			// 设置一个聚类需要的最大点数目
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	///////////////////////////显示1///////////////////////
	//std::vector<pcl::PointCloud<pcl::PointXYZ>> vCloud;
	//for (const auto& indices : cluster_indices) 
	//{
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	//	for (const auto& index : indices.indices)
	//		cloud_cluster->points.push_back(cloud->points[index]);

	//	vCloud.emplace_back(*cloud_cluster);

	//	cloud_cluster->width = cloud_cluster->points.size();
	//	cloud_cluster->height = 1;
	//	cloud_cluster->is_dense = true;

	//	// 使用 PCL 可视化工具显示点云
	//	pcl::visualization::CloudViewer viewer("Polyline Viewer");
	//	viewer.showCloud(cloud_cluster);

	//	// 等待直到视图关闭
	//	while (!viewer.wasStopped()) {}
	//}
	//
	
	// Create a new point cloud for colored clusters
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	colored_cloud->header = cloud->header;
	colored_cloud->width = cloud->width;
	colored_cloud->height = cloud->height;
	colored_cloud->is_dense = cloud->is_dense;

	int cluster_id = 0;

	for (const auto& indices : cluster_indices) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
		cluster_cloud->width = indices.indices.size();
		cluster_cloud->height = 1;
		cluster_cloud->points.resize(cluster_cloud->width * cluster_cloud->height);
		clusterCloud->points.resize(indices.indices.size());

		// Assign a unique color to this cluster
		pcl::PointXYZRGB color;
		color.r = (rand() % 256);
		color.g = (rand() % 256);
		color.b = (rand() % 256);

		for (size_t i = 0; i < indices.indices.size(); ++i) {
			pcl::PointXYZRGB& pt = cluster_cloud->points[i];
			pcl::PointXYZ& pt2 = clusterCloud->points[i];
			pt.x = cloud->points[indices.indices[i]].x; 
			pt.y = cloud->points[indices.indices[i]].y;
			pt.z = cloud->points[indices.indices[i]].z;
			pt2.x = cloud->points[indices.indices[i]].x;
			pt2.y = cloud->points[indices.indices[i]].y;
			pt2.z = cloud->points[indices.indices[i]].z;

			pt.r = color.r;
			pt.g = color.g;
			pt.b = color.b;
		}

		*colored_cloud += *cluster_cloud;

		// 多段线
#if ENABLE_DISPLAY
		// 遍历点云并顺序连接相邻的点
		double dSamplingStep = 0.5;
		double dAvg = CalcCloudDensity(clusterCloud);

		// 创建可视化器
		pcl::visualization::PCLVisualizer viewer("Line Viewer");
		for (size_t i = 0; i < cluster_cloud->points.size() - 1; ++i)
		{
			std::string line_id = "line_" + std::to_string(i);
			double dis = pcl::euclideanDistance(cluster_cloud->points[i], cluster_cloud->points[i + 1]);
			if (dis > /*dSamplingStep **/2* dAvg)
				continue;

			viewer.addLine(cluster_cloud->points[i], cluster_cloud->points[i + 1], line_id);
		}
		viewer.resetCamera();

		// 等待直到视图关闭
		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
		}
#endif
	}

	// Visualize the colored point cloud
#if ENABLE_DISPLAY
	pcl::visualization::PCLVisualizer viewer2("Cluster Viewer");
	viewer2.addPointCloud<pcl::PointXYZRGB>(colored_cloud, "colored_cloud");
	viewer2.resetCamera();

	while (!viewer2.wasStopped()) {
		viewer2.spinOnce(100);
	}
#endif
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

void GetSlice(pcl::PointCloud<pcl::PointXYZ>::Ptr& sliceCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Eigen::Vector4d& n, double delta)
{
	//double delta = 0.2;			
	std::vector<int> pointIdx;

	for (int i = 0; i < cloud->size(); ++i)
	{
		double dPtToPlane = n[0] * (*cloud)[i].x + n[1] * (*cloud)[i].y + n[2] * (*cloud)[i].z + n[3];
		double wr = dPtToPlane - delta;
		double wl = dPtToPlane + delta;
		if (wr * wl <= 0)
		{
			pointIdx.emplace_back(i);
		}
	}

	pcl::copyPointCloud(*cloud, pointIdx, *sliceCloud);
	// 调试用
	//pcl::PCDWriter writer;
	//writer.write("sliceCloud.pcd", *sliceCloud, false);
	// ****************************包围盒内点云显示******************************
#if ENABLE_DISPLAY
	pcl::visualization::PCLVisualizer viewer("CropBox Viewer");
	viewer.addPointCloud(sliceCloud);
	viewer.resetCamera();
	// 等待直到视图关闭
	while (!viewer.wasStopped()) 
	{
		viewer.spinOnce(100);
	}
#endif
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

pcl::PointCloud<pcl::PointXYZ>::Ptr TransformCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, Eigen::Matrix4d transformT)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZ>);
	cloudout->width = cloudIn->width;
	cloudout->height = cloudIn->height;
	cloudout->is_dense = cloudIn->is_dense;
	cloudout->points.resize(cloudIn->points.size());

	// 启用OpenMP并行化
#pragma omp parallel for 
	// 遍历原始点云中的每个点  
	for (int i = 0; i < cloudIn->points.size(); ++i)
	{
		// 创建一个4x1的齐次坐标向量（x, y, z, 1）  
		Eigen::Vector4d point_homogeneous(cloudIn->points[i].x, cloudIn->points[i].y, cloudIn->points[i].z, 1.0);

		// 应用变换矩阵到齐次坐标向量  
		Eigen::Vector4d transformed_point_homogeneous = transformT * point_homogeneous;

		// 提取变换后的3D坐标（x, y, z）  
		cloudout->points[i].x = transformed_point_homogeneous(0);
		cloudout->points[i].y = transformed_point_homogeneous(1);
		cloudout->points[i].z = transformed_point_homogeneous(2);
	}

	return cloudout;
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

	double dAvgDis = CalcCloudDensity(pc);

	// ****************************获取包围盒内的点云******************************
	// TEST CASE 01
	//Eigen::Vector3d center(-25.402, -14.551, -19.163);
	//Eigen::Vector3d n(0.577, 0.577, 0.577);

	// TEST CASE 02
	/*Eigen::Vector3d center(1.947, 3.896, -6.655);
	Eigen::Vector3d n(0.267, 0.535, 0.802);*/

	// TEST CASE 03
	//Eigen::Vector3d center(4.734, 9.470, 1.706);
	//Eigen::Vector3d n(0.267, 0.535, 0.802);

	// TEST CASE 04
	//Eigen::Vector3d center(38.996, -40.695, -0.002);
	//Eigen::Vector3d n(1, 2, 3);

	// TEST CASE 05
	//Eigen::Vector3d center(-53.674, -32.091, 0.002);
	//Eigen::Vector3d n(1, 1, 3);

	// TEST CASE 06
	//Eigen::Vector3d center(-95.978, 45.836, 0.005);
	//Eigen::Vector3d n(1, 3, 3);

	// TEST CASE 07
	//Eigen::Vector3d center(90.039, 85.834,-24.997);
	//Eigen::Vector3d n(2, 3, 3);

	// TEST CASE 08
	//Eigen::Vector3d center(-5.553, -34.169, -25.007);
	//Eigen::Vector3d n(1, 3, 1);

	// TEST CASE 09
	//Eigen::Vector3d center(-43.953, 23.834, -4.247);
	//Eigen::Vector3d n(3, 2, 1 );

	// TEST CASE 10
	//Eigen::Vector3d center(95.665, 72.443, -16.049);
	//Eigen::Vector3d n(10, 12, 20);

	// TEST CASE 11
	Eigen::Vector3d center(-95.978, 45.836, 0.005);
	Eigen::Vector3d n(1, 3, 3);

	Eigen::Vector3d z(0, 0, 1);
	Eigen::Vector3d axis = n.normalized().cross(z);
	double angle = acos(n.dot(z) / n.norm());

	Eigen::AngleAxisd rotation(angle, axis.normalized());
	Eigen::Matrix3d rotationMatrix = rotation.toRotationMatrix();

	// 使用旋转矩阵旋转点云
	Eigen::Matrix4d transMtx = Eigen::Matrix4d::Identity();
	transMtx.block<3, 3>(0, 0) = rotationMatrix;

	pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	rotatedCloud = TransformCloud(pc, transMtx);
	Eigen::Vector3d rotatedPt = rotationMatrix * center;

	// 得到包围盒内点云
	Eigen::Vector4d plane = CalcPlane(rotatedPt, z);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sliceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	GetSlice(sliceCloud, rotatedCloud, plane, dAvgDis*0.3);

	//pcl::PLYWriter writer;
	//writer.write("slice0820.ply", *sliceCloud, false);
	//std::cout << "保存成功！" << std::endl;
// 
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
	PointCoordinateType* preferredNormDir = new PointCoordinateType[3]{ 0.0, 0.0, 1.0 };
	PointCoordinateType* preferredUpDir = new PointCoordinateType[3]{ 0.0, -1.0, 0.0 };

	// TEST CASE 01
	///*preferredNormDir = new PointCoordinateType[3]{ 0.577, 0.577, 0.577 };
	//preferredUpDir = new PointCoordinateType[3]{ -0.408, -0.408, 0.816 };*/

	// TEST CASE 02
	///*preferredNormDir = new PointCoordinateType[3]{ 0.267, 0.535, 0.802 };
	//preferredUpDir = new PointCoordinateType[3]{ -0.359, -0.717, 0.598 };*/

	//preferredNormDir = new PointCoordinateType[3]{ 0.0, 0.0, 1.0 };
	//preferredUpDir = new PointCoordinateType[3]{ 0.0, -1.0, 0.0 };

	auto startOp = std::chrono::high_resolution_clock::now();
	CCPolyline* polyLine = ExtractFlatEnvelope(points, maxEdgeLength, preferredNormDir, preferredUpDir);
	auto endOp = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedOp = endOp - startOp;
	std::cout << "点云切片算法用时: " << elapsedOp.count() << " seconds" << std::endl;

	// ****************************保存数据******************************
	//pcl::PLYWriter writer;
	//writer.write("result.ply", *cloud, false);
	//std::cout << "保存成功！" << std::endl;

	system("pause");
	return 0;
}

