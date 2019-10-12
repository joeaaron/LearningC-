#pragma once
#include <cassert>

using namespace std;

namespace UF5 {

	class UnionFind {

	private:
		int* parent;     // parent[i]��ʾ��i��Ԫ����ָ��ĸ��ڵ�
		int* rank;		 // rank[i]��ʾ��iΪ���ļ�������ʾ�����Ĳ���
		int count;		 // ���ݸ���

	public:
		// ���캯��
		UnionFind(int count) {
			parent = new int[count];
			rank = new int[count];
			this->count = count;
			// ��ʼ��, ÿһ��parent[i]ָ���Լ�, ��ʾÿһ��Ԫ���Լ��Գ�һ������
			for (int i = 0; i < count; i++) {
				parent[i] = i;
				rank[i] = 1;
			}

		}

		// ��������
		~UnionFind() {
			delete[] parent;
			delete[] rank;
		}

		// ���ҹ���, ����Ԫ��p����Ӧ�ļ��ϱ��
		// O(h)���Ӷ�, hΪ���ĸ߶�
		int find(int p) {
			assert(p >= 0 && p < count);
			// path compression 1
			while (p != parent[p]) {
				parent[p] = parent[parent[p]];
				p = parent[p];
			}
			return p;
		}

		// �鿴Ԫ��p��Ԫ��q�Ƿ�����һ������
		// O(h)���Ӷ�, hΪ���ĸ߶�
		bool isConnected(int p, int q) {
			return find(p) == find(q);
		}

		// �ϲ�Ԫ��p��Ԫ��q�����ļ���
		// O(h)���Ӷ�, hΪ���ĸ߶�
		void unionElements(int p, int q) {

			int pRoot = find(p);
			int qRoot = find(q);

			if (pRoot == qRoot)
				return;

			// ��������Ԫ����������Ԫ�ظ�����ͬ�жϺϲ�����
			// ��Ԫ�ظ����ٵļ��Ϻϲ���Ԫ�ظ�����ļ�����
			if (rank[pRoot] < rank[qRoot]) {
				parent[pRoot] = qRoot;
			}
			else if (rank[qRoot] < rank[pRoot]){
				parent[qRoot] = pRoot;
			}
			else {
				parent[pRoot] = qRoot;
				rank[qRoot] += 1;
			}
		}
	};
}