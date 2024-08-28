#include <iostream>
#include <ctime>
#include <math.h>
#include <vector>

using namespace std;

// �����
struct Pos {
	int x;
	int y;
};

// �����
struct Edge 
{
	int x1, x2;
	int y1, y2;
	int a, b, c;
};

// ��������
struct Poly 
{
	// �߼�
	Edge Edges[100];
	int edgeNums = 0; //������
};

Poly poly;

// �󽻵����꣬�����A��B��C��ֱ�߷��� Ax + By + C = 0 �Ĳ������󽻵�������������˼���õ���
// ����ɲ鿴���ͣ��ò�����άֱ�߽���
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
		cout << "��" << p1 << "�ߺ͵�" << p2 << "��" << "�޽���" << endl;
	else {
		res.x = (C2 * B1 - C1 * B2) / m;
		res.y = (C1 * A2 - C2 * A1) / m;
	}
	return res;

}

// �жϵ��Ƿ����߶��ڣ���˼�����жϵ��Ƿ��ھ��ο���
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

// �ж���ʲô�����
bool Judge() 
{
	/*�������Ϣ*/
	for (int i = 0; i < poly.edgeNums; i++) {
		cout << "A��" << poly.Edges[i].a << "  " << "B��" << poly.Edges[i].b << "  " << "C��" << poly.Edges[i].c << endl;
	}

	/*�ж��Խ�*/
	Pos interPos;
	if (poly.edgeNums > 3)
	{
		for (int i = 0; i < poly.edgeNums; i++)
		{
			interPos = CrossPos(i, (i + 2) % poly.edgeNums);  // �ཻ���жϵ��Ƿ��ھ��η�Χ��
			if (JudgeInLine(interPos, poly.Edges[i]) && 
				JudgeInLine(interPos, poly.Edges[(i + 2) % poly.edgeNums])) 
			{
				cout << "�ö����Ϊ���ཻ�����" << endl;
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