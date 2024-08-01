
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
#include <pcl/visualization/cloud_viewer.h>

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
				assert(pointFlags[P.index] == POINT_NOT_USED); //we don't consider already used points!

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
	cloud->points.resize(nSize);
	for (unsigned int i = 0; i < nSize; ++i)
	{
		CCVector3 point;
		envVertices->getPoint(i, point);
		cloud->points.push_back(pcl::PointXYZ(point.x, point.y, point.z));
	}

	cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->is_dense = true;
	// 使用 PCL 可视化工具显示点云
	pcl::visualization::CloudViewer viewer("Polyline Viewer");
	viewer.showCloud(cloud);

	// 等待直到视图关闭
	while (!viewer.wasStopped()) {}

	return envelopePolyline;
}

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

	// ****************************切片算法******************************
	CCCoreLib::GenericIndexedCloudPersist* points = static_cast<CCCoreLib::GenericIndexedCloudPersist*>(&ccCloud);
	PointCoordinateType normDir[3] = { 0.0, 0.0, 1.0 };  // 初始化 normDir 数组，包含三个值
	PointCoordinateType upDir[3] = { 1.0, 0.0, 0.0 };	 // 初始化 upDir 数组，包含三个值
	PointCoordinateType* preferredNormDir = normDir;
	PointCoordinateType* preferredUpDir = upDir;
	PointCoordinateType maxEdgeLength = 0.002502;

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

