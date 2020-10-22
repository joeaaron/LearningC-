/*
********���·��-���ȶ���**********
*/
#include <iostream>
#include <windows.h>
#include <stack>
#include <Queue>
using namespace std;
const int N = 100;							 // ���еĸ������޸�
const int INF = 1e7;						 // �����10000000
int map[N][N], dist[N],  n, m;				 // n���еĸ�����mΪ���м�·�ߵ�����
bool flag[N];								 // ���s[i]����true��˵������i�Ѿ����뵽����S;���򶥵�i���ڼ���V-S

struct Node{
	int u, step;
	Node(){};
	Node(int a, int sp){
		u = a;
		step = sp;
	}
	bool operator < (const Node& a) const{
		return step > a.step;
	}
};

void Dijkstra(int st)
{
	priority_queue <Node> Q;				 // ���ȶ����Ż�
	Q.push(Node(st, 0));
	memset(flag, 0, sizeof(flag));			 // ��ʼ��flag����Ϊ0
	for (int i = 1; i <= n; i++)
		dist[i] = INF;						 // ��ʼ�����о���Ϊ�����

	dist[st] = 0;
	while (!Q.empty())
	{
		Node it = Q.top();					 // ���ȶ��е�ͷԪ��Ϊ��Сֵ
		Q.pop();
		int t = it.u;
		if (flag[t])						 // ˵���Ѿ��ҵ�����̾��룬�ý���Ƕ���������ظ�Ԫ��
			continue;
		flag[t] = 1;
		for (int i = 1; i <= n; i++)
		{
			if (!flag[i] && map[t][i]<INF){   // �ж��뵱ǰ���й�ϵ�ĵ㣬�����Լ����ܵ��Լ�
				if (dist[i]>dist[t] + map[t][i])
				{							  // ����뵱ǰ���ÿ�������̾���,�����ɳڲ���
					dist[i] = dist[t] + map[t][i];
					Q.push(Node(i, dist[i])); // �Ѹ��º����̾���ѹ�����ȶ��У�ע�⣺�����Ԫ�����ظ�
				}
			}
		}
	}

}

int main()
{
	int u, v, w, st;
	system("color 0d");
	cout << "��������еĸ�����" << endl; cin >> n;
	cout << "���������֮���·�ߵĸ���:" << endl; cin >> m;
	cout << "���������֮���·���Լ����룺" << endl;
	for (int i = 1; i <= n; i++)
		for (int j = 1; j <= n; j++)
		{
			map[i][j] = INF;				//��ʼ���ڽӾ���Ϊ�����
		}
	while (m--)
	{
		cin >> u >> v >> w;
		map[u][v] = min(map[u][v], w);		//�ڽӾ��󴢴棬������С�ľ���
	}
	cout << "������С�����ڵ�λ��:" << endl;;
	cin >> st;
	Dijkstra(st);
	cout << "С�����ڵ�λ��:" << st << endl;
	for (int i = 1; i <= n; i++)
	{
		cout << "С��:" << st << " - " << "Ҫȥ��λ��:" << i;
		if (dist[i] == INF)
			cout << "sorry,��·�ɴ�" << endl;
		else
			cout << " ��̾���Ϊ:" << dist[i] << endl;
	}

	system("pause");
	return 0;
}
/*
*******************************************
*/

