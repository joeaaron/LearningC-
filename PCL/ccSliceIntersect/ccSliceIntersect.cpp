#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

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

#define ENABLE_DISPLAY 1		// 定义一个宏，用于控制显示状态

typedef std::pair<pcl::PointXYZ, pcl::PointXYZ> LineSegment;

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

using CCPointCloud = CCCoreLib::GenericIndexedCloudPersist;
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

}

PointCloud::Ptr TransformCloud(const PointCloud::Ptr cloudIn, Eigen::Matrix4d transformT)
{
	PointCloud::Ptr cloudout(new PointCloud);
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

// Step 1: 计算点云密度
double ComputeDensity(PointCloud::Ptr cloud, int N, int m)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	std::vector<double> vDensities;
	for (int i = 0; i < N; ++i)
	{
		std::vector<int> pointIdxNKNSearch(m);
		std::vector<float> pointNKNSquaredDistance(m);
		pcl::PointXYZ searchPoint = (*cloud)[rand() % cloud->points.size()];

		if (kdtree.nearestKSearch(searchPoint, m, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			double density = 0.0;
			for (double dist : pointNKNSquaredDistance)
				density += sqrt(dist);

			vDensities.push_back(density / m);
		}
	}
	return std::accumulate(vDensities.begin(), vDensities.end(), 0.0) / vDensities.size();
}

// Step 2: 得到左右点云
void SlicePointCloud(
	PointCloud::Ptr lpoints,
	PointCloud::Ptr rpoints,
	PointCloud::Ptr cloud,
	double dPos, double delta)
{
	for (const auto& point : cloud->points)
	{
		if (point.z >= dPos - delta / 2 && point.z <= dPos)
		{
			lpoints->points.push_back(point);
		}
		else if (point.z > dPos && point.z <= dPos + delta / 2)
		{
			rpoints->points.push_back(point);
		}
	}

	std::cout << "Lpoints: " << lpoints->points.size() << ", Rpoints: " << rpoints->points.size() << std::endl;
}

// Step 3: 匹配 lpoint 和 rpoint 并计算交点
void FindCorres(pcl::PointCloud<pcl::PointXYZ>::Ptr sliceCloud,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr lpoints,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr rpoints,
	double dPos, double threshold)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(rpoints);

	for (const auto& lpoint : lpoints->points)
	{
		// 平面上的点直接加进去
		if (std::abs(lpoint.z - dPos) < 1e-4)
		{
			sliceCloud->points.push_back(lpoint);
			continue;
		}

		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);

		if (kdtree.nearestKSearch(lpoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			float dist = sqrt(pointNKNSquaredDistance[0]);
			if (dist < threshold)
			{
				pcl::PointXYZ rpoint = rpoints->points[pointIdxNKNSearch[0]];
				pcl::PointXYZ intersection;

				double dCoef1 = dPos - lpoint.z;
				double dCoef2 = rpoint.z - lpoint.z;

				if (fabs(dCoef2) < DBL_EPSILON) return;

				intersection.x = dCoef1 * (rpoint.x - lpoint.x) / dCoef2 + lpoint.x;
				intersection.y = dCoef1 * (rpoint.y - lpoint.y) / dCoef2 + lpoint.y;
				intersection.z = dPos;

				sliceCloud->points.push_back(intersection);
			}
		}
	}

#if ENABLE_DISPLAY
	pcl::visualization::PCLVisualizer viewer("SliceViewer");
	viewer.addPointCloud(sliceCloud);
	viewer.resetCamera();

	// 等待直到视图关闭
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
#endif

	std::cout << "Intersection points: " << sliceCloud->points.size() << std::endl;
}


void IntersectMethod(pcl::PointCloud<pcl::PointXYZ>::Ptr sliceCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	// TEST CASE 01
	//Eigen::Vector3d center(-95.978, 45.836, 0.005);
	//Eigen::Vector3d n(1, 3, 3);

	// TEST CASE 02
	Eigen::Vector3d center(80.770, 78.939, -12.500);
	Eigen::Vector3d n(0.394, 0.473, 0.788);

	Eigen::Vector3d z(0, 0, 1);
	Eigen::Vector3d axis = n.normalized().cross(z);
	double angle = acos(n.dot(z) / n.norm());

	Eigen::AngleAxisd rotation(angle, axis.normalized());
	Eigen::Matrix3d rotationMatrix = rotation.toRotationMatrix();

	// 使用旋转矩阵旋转点云
	Eigen::Matrix4d transMtx = Eigen::Matrix4d::Identity();
	transMtx.block<3, 3>(0, 0) = rotationMatrix;

	pcl::PointCloud<pcl::PointXYZ>::Ptr transCloud(new pcl::PointCloud<pcl::PointXYZ>);
	transCloud = TransformCloud(cloud, transMtx);
	Eigen::Vector3d rotatedPt = rotationMatrix * center;

	// Step 1: 计算点云密度
	double dDelta = ComputeDensity(transCloud, 100, 5);

	// Step 2: 得到左右点云 (假设我们沿着Z轴切片)
	pcl::PointCloud<pcl::PointXYZ>::Ptr lpoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr rpoints(new pcl::PointCloud<pcl::PointXYZ>);

	double dMaxDisToPlane = 2.000;  // 最大点至平面距离
	SlicePointCloud(lpoints, rpoints, transCloud, rotatedPt[2], dMaxDisToPlane/* 8 * dDelta*/);

	// Step 3: 匹配 lpoint 和 rpoint 并计算交点
	FindCorres(sliceCloud, lpoints, rpoints, rotatedPt[2], 2 * dDelta);
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

// 多义线连接：转CC二维点的方式
void SortPointsUsingCC(PointCloud::Ptr sortedCloud, const PointCloud::Ptr& cloud)
{
	PointCoordinateType maxEdgeLength = CalcBoxDis(cloud) / 100.0;
	std::vector<Vertex2D> ccPoints;
	double dZ = cloud->points[0].z;
	// PCL点云转CC点集(2D)
	ccPoints.reserve(cloud->points.size());
	for (const auto& point : cloud->points)
	{
		ccPoints.emplace_back(Vertex2D(point.x, point.y));
	}

	Hull2D hullPoints;
	if (!ExtractConcaveHull2D(ccPoints,
		hullPoints,
		0.001 * maxEdgeLength, 0))
	{
		return;
	}

	// 遍历顶点，并将其添加到 PCL 点云对象中
	int nSize = hullPoints.size();
	sortedCloud->points.reserve(nSize);
	for (Hull2D::const_iterator it = hullPoints.begin(); it != hullPoints.end(); ++it)
	{
		sortedCloud->points.push_back(pcl::PointXYZ((*it)->x, (*it)->y, dZ));
	}

#if ENABLE_DISPLAY
	pcl::visualization::PCLVisualizer viewer("SortedViewer");
	viewer.addPointCloud(sortedCloud);
	viewer.resetCamera();

	// 等待直到视图关闭
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
#endif
}

// 多义线连接：凸包结合凹包
void SortPointsUsingHull(PointCloud::Ptr sortedPoints, const PointCloud::Ptr& cloud)
{
	// Step 1: Compute Convex Hull
	PointCloud::Ptr convex_hull_cloud(new PointCloud());
	pcl::ConvexHull<pcl::PointXYZ> convex_hull;
	convex_hull.setInputCloud(cloud);
	convex_hull.reconstruct(*convex_hull_cloud);

	// Step 2: Compute Concave Hull
	PointCloud::Ptr concave_hull_cloud(new PointCloud());
	pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
	concave_hull.setInputCloud(cloud);
	concave_hull.setAlpha(7.5); // Alpha value controls concavity
	concave_hull.reconstruct(*concave_hull_cloud);

	// Add convex hull points first (they form the outer boundary)
	for (const auto& point : convex_hull_cloud->points) {
		sortedPoints->points.push_back(point);
	}

	// Insert concave hull points into the ordered list based on proximity to the convex hull
	for (const auto& point : concave_hull_cloud->points) {
		// Find the closest point in the ordered convex hull points
		double min_distance = std::numeric_limits<double>::max();
		size_t insert_position = 0;

		for (size_t i = 0; i < sortedPoints->points.size(); ++i) {
			double distance = std::sqrt(std::pow(point.x - sortedPoints->points[i].x, 2) +
				std::pow(point.y - sortedPoints->points[i].y, 2) +
				std::pow(point.z - sortedPoints->points[i].z, 2));
			if (distance < min_distance) {
				min_distance = distance;
				insert_position = i;
			}
		}

		// Insert the concave point near its closest convex point
		sortedPoints->points.insert(sortedPoints->points.begin() + insert_position, point);
	}

#if ENABLE_DISPLAY
	pcl::visualization::PCLVisualizer viewer("SortedViewer");
	viewer.addPointCloud(sortedPoints);
	viewer.resetCamera();

	// 等待直到视图关闭
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
#endif
}

// 多义线连接：区域生长
void SortPointsUsingKDTree(PointCloud::Ptr sortedPoints, const PointCloud::Ptr& cloud)
{
	// 构建 k-d Tree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	// 计算点云密度
	double dAvgDis = 0.0;
	int nNum = 0;
	for (int i = 0; i < cloud->size(); ++i)
	{
		std::vector<int> indiceId;
		std::vector<float> disSquare;

		if (kdtree.nearestKSearch(cloud->points[i], 2, indiceId, disSquare) > 0)
		{
			dAvgDis += sqrt(disSquare[1]);
			nNum++;
		}
	}
	dAvgDis /= nNum;

	// 选择起始点，例如选择最左下角的点
	pcl::PointXYZ startPoint = cloud->points[0];  // 假设选择第一个点作为起始点
	sortedPoints->points.push_back(startPoint);

	std::vector<bool> used(cloud->points.size(), false);
	used[0] = true;

	pcl::PointXYZ currentPoint = startPoint;

	for (size_t i = 1; i < cloud->points.size(); ++i)
	{
		float searchRadius = dAvgDis;						//0.1f：搜索半径，可根据点云密度调整
		bool foundValidPoint = false;

		while (!foundValidPoint)
		{
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			// kd-tree 半径搜索，控制搜索范围
			if (kdtree.radiusSearch(currentPoint, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= 0)
			{
				searchRadius *= 2;
			}
			else
			{
				int nearestIdx = -1;
				float minDistance = std::numeric_limits<float>::max();

				for (int j = 0; j < pointIdxRadiusSearch.size(); ++j)
				{
					int idx = pointIdxRadiusSearch[j];
					if (!used[idx] && pointRadiusSquaredDistance[j] < minDistance)
					{
						nearestIdx = idx;
						minDistance = pointRadiusSquaredDistance[j];
					}
				}
				// 如果找到合适的最近点
				if (nearestIdx != -1)
				{
					pcl::PointXYZ nearestPoint = cloud->points[nearestIdx];
					sortedPoints->points.push_back(nearestPoint);
					used[nearestIdx] = true;
					currentPoint = nearestPoint;
					foundValidPoint = true;
				}
				else
				{
					searchRadius *= 2;		// 动态调整半径
				}
			}
		}
	}

#if ENABLE_DISPLAY
	pcl::visualization::PCLVisualizer viewer("SortedViewer");
	viewer.addPointCloud(sortedPoints);
	viewer.resetCamera();

	// 等待直到视图关闭
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
#endif
}

double CalculateAngle(const pcl::PointXYZ& origin, const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
{
	double angle1 = atan2(p1.y - origin.y, p1.x - origin.x);
	double angle2 = atan2(p2.y - origin.y, p2.x - origin.x);
	double angle = angle2 - angle1;

	// 确保角度在 [0, 2 * PI] 范围内
	if (angle < 0) angle += 2 * M_PI;
	return angle;
}

// 多义线连接：区域生长带法向量
void SortPointsUsingKDTreeEx(PointCloud::Ptr sortedPoints, const PointCloud::Ptr& cloud)
{
	// 构建 k-d Tree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	// 计算点云密度
	double dAvgDis = 0.0;
	int nNum = 0;
	for (int i = 0; i < cloud->size(); ++i) {
		std::vector<int> indiceId;
		std::vector<float> disSquare;

		if (kdtree.nearestKSearch(cloud->points[i], 2, indiceId, disSquare) > 0) {
			dAvgDis += sqrt(disSquare[1]);
			nNum++;
		}
	}
	dAvgDis /= nNum;

	// 选择起始点，例如选择最左下角的点
	pcl::PointXYZ startPoint = cloud->points[0];  // 假设选择第一个点作为起始点
	sortedPoints->points.push_back(startPoint);

	std::vector<bool> used(cloud->points.size(), false);
	used[0] = true;

	pcl::PointXYZ currentPoint = startPoint;
	pcl::PointXYZ previousPoint = startPoint;  // 用于计算方向

	for (size_t i = 1; i < cloud->points.size(); ++i) {
		float searchRadius = dAvgDis;
		bool foundValidPoint = false;

		while (!foundValidPoint) {
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			// kd-tree 半径搜索，控制搜索范围
			if (kdtree.radiusSearch(currentPoint, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= 0)
			{
				searchRadius *= 2;
			}
			else
			{
				int bestIdx = -1;
				double minAngle = std::numeric_limits<float>::max();

				for (int j = 0; j < pointIdxRadiusSearch.size(); ++j)
				{
					int idx = pointIdxRadiusSearch[j];
					if (!used[idx])
					{
						double angle = CalculateAngle(previousPoint, currentPoint, cloud->points[idx]);
						if (angle < minAngle)   // 角度变化最小 
						{
							minAngle = angle;
							bestIdx = idx;
						}
					}
				}

				// 如果找到合适的点（按方向排序）
				if (bestIdx != -1) {
					pcl::PointXYZ nextPoint = cloud->points[bestIdx];
					sortedPoints->points.push_back(nextPoint);
					used[bestIdx] = true;
					previousPoint = currentPoint;
					currentPoint = nextPoint;
					foundValidPoint = true;
				}
				else
				{
					searchRadius *= 2;  // 动态调整半径
				}
			}
		}
	}
}

// 计算向量叉积
float crossProduct(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3) {
	return (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
}

// 判断点是否在区间内
bool isBetween(float v, float min, float max) {
	return (v >= min && v <= max);
}

// 判断两条线段是否相交
bool doLineSegmentsIntersect(const LineSegment& seg1, const LineSegment& seg2) {
	const pcl::PointXYZ& p1 = seg1.first;
	const pcl::PointXYZ& p2 = seg1.second;
	const pcl::PointXYZ& p3 = seg2.first;
	const pcl::PointXYZ& p4 = seg2.second;

	float d1 = crossProduct(p3, p4, p1);
	float d2 = crossProduct(p3, p4, p2);
	float d3 = crossProduct(p1, p2, p3);
	float d4 = crossProduct(p1, p2, p4);

	// 判断是否相交
	if ((d1 * d2 < 0) && (d3 * d4 < 0)) {
		return true;
	}

	// 判断是否共线
	if (d1 == 0 && d2 == 0 && d3 == 0 && d4 == 0) {
		return isBetween(p1.x, std::min(p3.x, p4.x), std::max(p3.x, p4.x)) &&
			isBetween(p1.y, std::min(p3.y, p4.y), std::max(p3.y, p4.y)) &&
			isBetween(p1.z, std::min(p3.z, p4.z), std::max(p3.z, p4.z)) &&
			isBetween(p2.x, std::min(p3.x, p4.x), std::max(p3.x, p4.x)) &&
			isBetween(p2.y, std::min(p3.y, p4.y), std::max(p3.y, p4.y)) &&
			isBetween(p2.z, std::min(p3.z, p4.z), std::max(p3.z, p4.z));
	}

	return false;
}

// 检测多边形是否自相交的函数 (简单示例)
bool IsSelfIntersecting(const PointCloud::Ptr& cloud)
{
	size_t n = cloud->points.size();
	for (size_t i = 0; i < n; ++i)
	{
		for (size_t j = i + 2; j < n; ++j)
		{
			if (i == 0 && j == n - 1)
				continue;

			// 检测线段 (i, i+1) 与 (j, j+1) 是否相交
			pcl::PointXYZ p1 = cloud->points[i];
			pcl::PointXYZ p2 = cloud->points[(i + 1) % n];
			pcl::PointXYZ p3 = cloud->points[j];
			pcl::PointXYZ p4 = cloud->points[(j + 1) % n];

			float d1 = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
			float d2 = (p2.x - p1.x) * (p4.y - p1.y) - (p2.y - p1.y) * (p4.x - p1.x);
			float d3 = (p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x);
			float d4 = (p4.x - p3.x) * (p2.y - p3.y) - (p4.y - p3.y) * (p2.x - p3.x);

			if ((d1 > 0 && d2 < 0 || d1 < 0 && d2 > 0) && (d3 > 0 && d4 < 0 || d3 < 0 && d4 > 0))
				return true;
		}
	}
	return false;
}

// 多义线连接：区域生长自交检测
void SortPointsUsingKDTreeIntersect(PointCloud::Ptr sortedPoints, const PointCloud::Ptr& cloud)
{
	// 构建 k-d Tree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	// 计算点云密度
	double dAvgDis = 0.0;
	int nNum = 0;
	for (int i = 0; i < cloud->size(); ++i)
	{
		std::vector<int> indiceId;
		std::vector<float> disSquare;

		if (kdtree.nearestKSearch(cloud->points[i], 2, indiceId, disSquare) > 0)
		{
			dAvgDis += sqrt(disSquare[1]);
			nNum++;
		}
	}
	dAvgDis /= nNum;

	// 选择起始点，例如选择最左下角的点
	pcl::PointXYZ startPoint = cloud->points[0];  // 假设选择第一个点作为起始点
	sortedPoints->points.push_back(startPoint);

	std::vector<bool> used(cloud->points.size(), false);
	used[0] = true;

	pcl::PointXYZ currentPoint = startPoint;
	std::vector<LineSegment> sortedLineSegments;

	for (size_t i = 1; i < cloud->points.size(); ++i)
	{
		float searchRadius = dAvgDis;						//0.1f：搜索半径，可根据点云密度调整
		bool foundValidPoint = false;

		while (!foundValidPoint)
		{
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			// kd-tree 半径搜索，控制搜索范围
			if (kdtree.radiusSearch(currentPoint, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= 0)
			{
				searchRadius *= 2;
			}
			else
			{
				int nearestIdx = -1;
				float minDistance = std::numeric_limits<float>::max();

				for (int j = 0; j < pointIdxRadiusSearch.size(); ++j)
				{
					int idx = pointIdxRadiusSearch[j];
					if (!used[idx] && pointRadiusSquaredDistance[j] < minDistance)
					{
						nearestIdx = idx;
						minDistance = pointRadiusSquaredDistance[j];
					}
				}
				// 如果找到合适的最近点
				if (nearestIdx != -1)
				{
					pcl::PointXYZ nearestPoint = cloud->points[nearestIdx];
					sortedPoints->points.push_back(nearestPoint);
					used[nearestIdx] = true;
					currentPoint = nearestPoint;

					// 检测自相交
					if (IsSelfIntersecting(sortedPoints))
					{
						// 自相交处理逻辑，比如回退或者重新计算
						sortedPoints->points.pop_back();
						used[nearestIdx] = false;
						currentPoint = sortedPoints->points.back(); // 回退到上一个有效点
						break; // 重新搜索
					}
					foundValidPoint = true;
				}
				else
				{
					searchRadius *= 2;		// 动态调整半径
				}
			}
		}
	}

#if ENABLE_DISPLAY
	pcl::visualization::PCLVisualizer viewer("SortedViewer");
	viewer.addPointCloud(sortedPoints);
	viewer.resetCamera();

	// 等待直到视图关闭
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
#endif
}


std::tuple<double, double> CalAvgStd(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	size_t numPoints = cloud->points.size();

	if (numPoints < 2)
	{
		return std::make_tuple(NAN, NAN);
		std::cerr << "Not enough points to compute distances." << std::endl;
	}

	std::vector<double> vDis;
	for (int i = 0; i < numPoints; ++i)
	{
		size_t nextIndex = (i + 1) % numPoints; // 最后一个点与第一个点相连
		const pcl::PointXYZ& pt1 = cloud->points[i];
		const pcl::PointXYZ& pt2 = cloud->points[nextIndex];
		auto distance = pcl::euclideanDistance(pt1, pt2);
		vDis.emplace_back(distance);

	}
	double dAvg = std::accumulate(vDis.begin(), vDis.end(), 0.0) / vDis.size();
	double dSqu = std::inner_product(vDis.begin(), vDis.end(), vDis.begin(), 0.0f);
	double dStd = std::sqrt(dSqu / vDis.size() - dAvg * dAvg);

	return std::make_tuple(dAvg, dStd);
}

void CurveReconstruct1(const PointCloud::Ptr& cloud)
{
	// 计算均值和标准差
	const auto& [mean, stddev] = CalAvgStd(cloud);

	// 欧几里德聚类提取器
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(mean);						// 设置近邻搜索的搜索半径
	ec.setMinClusterSize(3);							// 设置一个聚类需要的最少点数目
	ec.setMaxClusterSize(100000);						// 设置一个聚类需要的最大点数目
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(clusterIndices);

	std::vector<PointCloud::Ptr> vCloud;
	for (const auto& indices : clusterIndices)
	{
		PointCloud::Ptr cloudCluster(new PointCloud);
		for (const auto& index : indices.indices)
			cloudCluster->points.push_back(cloud->points[index]);

		// 进行点排序
		PointCloud::Ptr sortdCloud(new PointCloud);
		//SortPointsUsingHull(sortdCloud, cloudCluster);
		//SortPointsUsingCC(sortdCloud, cloudCluster);
		//SortPointsUsingKDTree(sortdCloud, cloudCluster);
		SortPointsUsingKDTreeEx(sortdCloud, cloudCluster);
		//SortPointsUsingKDTreeIntersect(sortdCloud, cloudCluster);

		// 创建可视化器
		pcl::visualization::PCLVisualizer viewer("Line Viewer");

		// 遍历点云并顺序连接相邻的点
		double dSamplingStep = 0.5;
		for (size_t i = 0; i < sortdCloud->points.size() - 1; ++i)
		{
			std::string line_id = "line_" + std::to_string(i);
			double dis = pcl::euclideanDistance(sortdCloud->points[i], sortdCloud->points[i + 1]);
			if (dis > dSamplingStep * mean)
				continue;

			viewer.addLine(sortdCloud->points[i], sortdCloud->points[i + 1], line_id);
		}
		// 连接最后一个点到第一个点，形成闭环
		double dis = pcl::euclideanDistance(sortdCloud->points.back(), sortdCloud->points.front());
		if (dis <= dSamplingStep * mean)
			viewer.addLine(sortdCloud->points.back(), sortdCloud->points.front(), "line_close");

		viewer.resetCamera();

		// 运行可视化器
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
		}

		vCloud.emplace_back(cloudCluster);
	}
}

void CurveReconstruct2(const PointCloud::Ptr& cloud)
{
	// Step 1: Extract line features using RANSAC
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a line model for the given dataset.");
		return;
	}

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.filter(*line_cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(line_cloud);

	std::vector<pcl::PointIndices> line_cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02);
	ec.setMinClusterSize(50);
	ec.setMaxClusterSize(10000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(line_cloud);
	ec.extract(line_cluster_indices);

	for (const auto& indices : line_cluster_indices)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (const auto& index : indices.indices)
			cloud_cluster->points.push_back(line_cloud->points[index]);

		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
	}

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL Viewer"));
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, "original cloud");
	viewer->addPointCloud<pcl::PointXYZ>(line_cloud, "line cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}

int main(int argc, char** argv)
{
	PointCloud::Ptr cloud(new PointCloud);

	if (pcl::io::loadPCDFile<PointT>("pmt.pcd", *cloud) == -1)		
	{
		PCL_ERROR("点云读取失败 \n");
		return (-1);
	}
	cout << "Points num = " << cloud->points.size() << std::endl;
	auto startOp = std::chrono::high_resolution_clock::now();

	PointCloud::Ptr sliceCloud(new PointCloud);
	
	// 相交法得到切片点云
	IntersectMethod(sliceCloud, cloud);	
	// 多义线连接
	CurveReconstruct1(sliceCloud);				

	return 0;
}