#include <iostream>
#include <ctime>
#include <math.h>
#include <vector>
#include <Eigen/Dense>

using namespace std;

// 定义边
struct Edge 
{
	Eigen::Vector2d pt1;
	Eigen::Vector2d pt2;
};

// 求交点坐标，这里的A、B、C是直线方程 Ax + By + C = 0 的参数，求交点是利用向量叉乘计算得到的
// 具体可查看博客：用叉积求二维直线交点
// https://www.jianshu.com/p/3468c9967fc7
Eigen::Vector2d CrossPos(Edge p1, Edge p2)
{
	Eigen::Vector2d A = p1.pt1;
	Eigen::Vector2d B = p1.pt2;
	Eigen::Vector2d C = p2.pt1;
	Eigen::Vector2d D = p2.pt2;

	// Compute direction vectors
	Eigen::Vector2d AB = B - A;
	Eigen::Vector2d CD = D - C;
	Eigen::Vector2d AC = C - A;

	// Compute the determinant
	double denom = AB.x() * CD.y() - AB.y() * CD.x();

	// Initialize a default return value (optional, could be any value indicating failure)
	Eigen::Vector2d result(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());

	// Check if lines are parallel
	if (denom == 0) {
		// Lines are parallel, return a default value or handle error as appropriate
		return result;
	}

	// Compute intersection point parameters
	double t = (AC.x() * CD.y() - AC.y() * CD.x()) / denom;
	double u = (AC.x() * AB.y() - AC.y() * AB.x()) / denom;

	// Check if the intersection point is within the bounds of the segments
	if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
	{
		result = A + t * AB;
	}

	// Else, 'result' will contain the default error value
	return result;
}

// 判断点是否在线段内，意思就是判断点是否在矩形框内
bool JudgeInLine(Eigen::Vector2d pos, Edge edge)
{
	int maxX = edge.pt1.x() >= edge.pt2.x() ? edge.pt1.x() : edge.pt2.x();
	int minX = edge.pt1.x() <= edge.pt2.x() ? edge.pt1.x() : edge.pt2.x();
	int maxY = edge.pt1.y() >= edge.pt2.y() ? edge.pt1.y() : edge.pt2.y();
	int minY = edge.pt1.y() <= edge.pt2.y() ? edge.pt1.y() : edge.pt2.y();
	if (pos.x() <= maxX && pos.x() >= minX && pos.y() <= maxY && pos.y() >= minY) {
		return true;
	}
	return false;
}

// 判断是什么多边形
bool Judge(const std::vector<Edge>& poly)
{
	/*判断自交*/
	Eigen::Vector2d interPos;
	if (poly.size() > 3)
	{
		for (int i = 0; i < poly.size(); i++)
		{
			interPos = CrossPos(poly[i], poly[(i + 2) % poly.size()]);  // 相交后判断点是否在矩形范围内
			if (JudgeInLine(interPos, poly[i]) &&
				JudgeInLine(interPos, poly[(i + 2) % poly.size()]))
			{
				cout << "该多边形为自相交多边形" << endl;
				return false;
			}
		}
	}

	return true;
}


struct LineSegment{
	Eigen::Vector2d p1, p2;

	LineSegment(const Eigen::Vector2d& _p1, const Eigen::Vector2d& _p2) : p1(_p1), p2(_p2) {}
};

// 叉积计算
double cross(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
	return v1.x() * v2.y() - v1.y() * v2.x();
}

// 检查两条线段是否相交
bool doIntersect(const LineSegment& seg1, const LineSegment& seg2) 
{
	Eigen::Vector2d pq = seg1.p2 - seg1.p1;
	Eigen::Vector2d pr = seg2.p1 - seg1.p1;
	Eigen::Vector2d ps = seg2.p2 - seg1.p1;

	double d1 = cross(pq, pr);
	double d2 = cross(pq, ps);

	Eigen::Vector2d rs = seg2.p2 - seg2.p1;
	Eigen::Vector2d rp = seg1.p1 - seg2.p1;
	Eigen::Vector2d rq = seg1.p2 - seg2.p1;

	double d3 = cross(rs, rp);
	double d4 = cross(rs, rq);

	return (d1 * d2 < 0) && (d3 * d4 < 0);
}

int main()
{
	//std::vector<Edge> poly;
	//Judge(poly);

	std::vector<Eigen::Vector2d> points = {
	   Eigen::Vector2d(0, 0),
	   Eigen::Vector2d(1, 1),
	   Eigen::Vector2d(2, 0),
	   Eigen::Vector2d(1, -1),
	   Eigen::Vector2d(3, 3)		 // 试图添加的点
	};

	vector<LineSegment> convexHullEdges;

	// 初始化凸包的第一条边
	convexHullEdges.emplace_back(points[0], points[1]);

	// 添加每一个新的点，并检测是否产生自相交
	for (size_t i = 2; i < points.size(); ++i) 
	{
		Eigen::Vector2d& newPoint = points[i];

		// 新边
		LineSegment newEdge(convexHullEdges.back().p2, newPoint);

		// 检测新边与已有边集是否相交（跳过相邻边）
		bool isIntersecting = false;
		for (size_t j = 0; j < convexHullEdges.size() - 1; ++j) 
		{
			// 检测当前边是否相邻
			if (convexHullEdges[j].p2 == newEdge.p1 || convexHullEdges[j].p1 == newEdge.p2) {
				continue; // 跳过相邻边
			}
			if (doIntersect(newEdge, convexHullEdges[j])) {
				isIntersecting = true;
				break;
			}
		}

		if (isIntersecting) {
			cout << "Point (" << newPoint.x() << ", " << newPoint.y() << ") causes self-intersection. Skipping." << endl;
		}
		else {
			convexHullEdges.push_back(newEdge);
			cout << "Point (" << newPoint.x() << ", " << newPoint.y() << ") added to convex hull." << endl;
		}
	}

	cout << "Final convex hull edges:" << endl;
	for (const auto& edge : convexHullEdges) {
		cout << "(" << edge.p1.x() << ", " << edge.p1.y() << ") -> ("
			<< edge.p2.x() << ", " << edge.p2.y() << ")" << endl;
	}

	return 0;
}