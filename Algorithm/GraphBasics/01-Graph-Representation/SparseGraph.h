#pragma once
#include <iostream>
#include <vector>
#include <cassert>

using namespace std;

// ϡ��ͼ - �ڽӱ�
class SparseGraph {

private:
	int n, m;       // �ڵ����ͱ���
	bool directed;  // �Ƿ�Ϊ����ͼ
	vector<vector<int>> g;  // ͼ�ľ�������

public:
	// ���캯��
	SparseGraph(int n, bool directed) {
		assert(n >= 0);
		this->n = n;
		this->m = 0;    // ��ʼ��û���κα�
		this->directed = directed;
		// g��ʼ��Ϊn���յ�vector, ��ʾÿһ��g[i]��Ϊ��, ��û���κͱ�
		g = vector<vector<int>>(n, vector<int>());
	}

	~SparseGraph() { }

	int V() { return n; } // ���ؽڵ����
	int E() { return m; } // ���رߵĸ���

	// ��ͼ�����һ����
	void addEdge(int v, int w) {

		assert(v >= 0 && v < n);
		assert(w >= 0 && w < n);

		g[v].push_back(w);
		if (v != w && !directed)
			g[w].push_back(v);

		m++;
	}

	// ��֤ͼ���Ƿ��д�v��w�ı�
	bool hasEdge(int v, int w) {

		assert(v >= 0 && v < n);
		assert(w >= 0 && w < n);

		for (int i = 0; i < g[v].size(); i++)
			if (g[v][i] == w)
				return true;
		return false;
	}
};
}