#include <iostream>
#include <ctime>
#include <math.h>
#include <vector>

using namespace std;

// 定义点
struct Pos {
	int x;
	int y;
};

// 定义边
struct Edge 
{
	int x1, x2;
	int y1, y2;
	int a, b, c;
};

// 定义多边形
struct Poly 
{
	// 边集
	Edge Edges[100];
	int edgeNums = 0; //边数量
};

Poly poly;

// 求交点坐标，这里的A、B、C是直线方程 Ax + By + C = 0 的参数，求交点是利用向量叉乘计算得到的
// 具体可查看博客：用叉积求二维直线交点
// https://www.jianshu.com/p/3468c9967fc7
Pos CrossPos(int p1, int p2) {
	Pos res;
	int A1 = poly.Edges[p1].a;
	int B1 = poly.Edges[p1].b;
	int A2 = poly.Edges[p2].a;
	int B2 = poly.Edges[p2].b;
	int C1 = poly.Edges[p1].c;
	int C2 = poly.Edges[p2].c;

	int m = A1 * B2 - A2 * B1;
	if (m == 0)
		cout << "第" << p1 << "边和第" << p2 << "边" << "无交点" << endl;
	else {
		res.x = (C2 * B1 - C1 * B2) / m;
		res.y = (C1 * A2 - C2 * A1) / m;
	}
	return res;

}

// 判断点是否在线段内，意思就是判断点是否在矩形框内
bool JudgeInLine(Pos pos, Edge edge)
{
	int maxX = edge.x1 >= edge.x2 ? edge.x1 : edge.x2;
	int minX = edge.x1 <= edge.x2 ? edge.x1 : edge.x2;
	int maxY = edge.y1 >= edge.y2 ? edge.y1 : edge.y2;
	int minY = edge.y1 <= edge.y2 ? edge.y1 : edge.y2;
	if (pos.x <= maxX && pos.x >= minX && pos.y <= maxY && pos.y >= minY) {
		return true;
	}
	return false;
}

// 判断是什么多边形
bool Judge() 
{
	/*输出边信息*/
	for (int i = 0; i < poly.edgeNums; i++) {
		cout << "A：" << poly.Edges[i].a << "  " << "B：" << poly.Edges[i].b << "  " << "C：" << poly.Edges[i].c << endl;
	}

	/*判断自交*/
	Pos interPos;
	if (poly.edgeNums > 3)
	{
		for (int i = 0; i < poly.edgeNums; i++)
		{
			interPos = CrossPos(i, (i + 2) % poly.edgeNums);  // 相交后判断点是否在矩形范围内
			if (JudgeInLine(interPos, poly.Edges[i]) && 
				JudgeInLine(interPos, poly.Edges[(i + 2) % poly.edgeNums])) 
			{
				cout << "该多边形为自相交多边形" << endl;
				return false;
			}
		}
	}

	return true;
}

int main()
{
	return 0;
}