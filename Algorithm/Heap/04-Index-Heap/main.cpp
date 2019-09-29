#include <iostream>
#include <cassert>
#include "SortTestHelper.h"

using namespace std;

// ���������
template<typename Item>
class IndexMaxHeap {

private:
	Item* data;     // ����������е�����
	int* indexes;   // ����������е�����

	int count;
	int capacity;

	// ��������, ����֮��ıȽϸ���data�Ĵ�С���бȽ�, ��ʵ�ʲ�����������
	void shiftUp(int k) {

		while (k > 1 && data[indexes[k / 2]] < data[indexes[k]]) {
			swap(indexes[k / 2], indexes[k]);
			k /= 2;
		}
	}

	// ��������, ����֮��ıȽϸ���data�Ĵ�С���бȽ�, ��ʵ�ʲ�����������
	void shiftDown(int k) {

		while (2 * k <= count) {
			int j = 2 * k;
			if (j + 1 <= count && data[indexes[j + 1]] > data[indexes[j]])
				j += 1;

			if (data[indexes[k]] >= data[indexes[j]])
				break;

			swap(indexes[k], indexes[j]);
			k = j;
		}
	}

public:
	// ���캯��, ����һ���յ�������, ������capacity��Ԫ��
	IndexMaxHeap(int capacity) {

		data = new Item[capacity + 1];
		indexes = new int[capacity + 1];

		count = 0;
		this->capacity = capacity;
	}

	~IndexMaxHeap() {
		delete[] data;
		delete[] indexes;
	}

	// �����������е�Ԫ�ظ���
	int size() {
		return count;
	}

	// ����һ������ֵ, ��ʾ���������Ƿ�Ϊ��
	bool isEmpty() {
		return count == 0;
	}

	// ������������в���һ���µ�Ԫ��, ��Ԫ�ص�����Ϊi, Ԫ��Ϊitem
	// �����i���û�����,�Ǵ�0������
	void insert(int i, Item item) {
		assert(count + 1 <= capacity);
		assert(i + 1 >= 1 && i + 1 <= capacity);

		i += 1;
		data[i] = item;
		indexes[count + 1] = i;
		count++;

		shiftUp(count);
	}

	// �������������ȡ���Ѷ�Ԫ��, �������������洢���������
	Item extractMax() {
		assert(count > 0);

		Item ret = data[indexes[1]];
		swap(indexes[1], indexes[count]);
		count--;
		shiftDown(1);
		return ret;
	}

	// �������������ȡ���Ѷ�Ԫ�ص�����
	int extractMaxIndex() {
		assert(count > 0);

		int ret = indexes[1] - 1;
		swap(indexes[1], indexes[count]);
		count--;
		shiftDown(1);
		return ret;
	}

	// ��ȡ����������еĶѶ�Ԫ��
	Item getMax() {
		assert(count > 0);
		return data[indexes[1]];
	}

	// ��ȡ����������еĶѶ�Ԫ�ص�����
	int getMaxIndex() {
		assert(count > 0);
		return indexes[1] - 1;
	}

	// ��ȡ���������������Ϊi��Ԫ��
	Item getItem(int i) {
		assert(i + 1 >= 1 && i + 1 <= capacity);
		return data[i + 1];
	}

	// �����������������Ϊi��Ԫ���޸�ΪnewItem
	void change(int i, Item newItem) {

		i += 1;
		data[i] = newItem;

		// �ҵ�indexes[j] = i, j��ʾdata[i]�ڶ��е�λ��
		// ֮��shiftUp(j), ��shiftDown(j)
		for (int j = 1; j <= count; j++)
			if (indexes[j] == i) {
				shiftUp(j);
				shiftDown(j);
				return;
			}
	}

	// �����������е���������index
	// ע��:�������������в���Ԫ���Ժ�, ������extract������Ч
	bool testIndexes() {

		int* copyIndexes = new int[count + 1];

		for (int i = 0; i <= count; i++)
			copyIndexes[i] = indexes[i];

		copyIndexes[0] = 0;
		std::sort(copyIndexes, copyIndexes + count + 1);

		// �ڶ��������е��������������, Ӧ��������1...count��count������
		bool res = true;
		for (int i = 1; i <= count; i++)
			if (copyIndexes[i - 1] + 1 != copyIndexes[i]) {
				res = false;
				break;
			}

		delete[] copyIndexes;

		if (!res) {
			cout << "Error!" << endl;
			return false;
		}

		return true;
	}
};

// ʹ����������ѽ��ж�����, ����֤���ǵ���������ѵ���ȷ��
// ��������ѵ���Ҫ���ò�����������, ����������ʹ������ֻ��Ϊ����֤���ǵ����������ʵ�ֵ���ȷ��
// �ں�����ͼ����, ��������С�������㷨, �������·���㷨, ���Ƕ���Ҫʹ�������ѽ����Ż�:)
template<typename T>
void heapSortUsingIndexMaxHeap(T arr[], int n) {

	IndexMaxHeap<T> indexMaxHeap = IndexMaxHeap<T>(n);
	for (int i = 0; i < n; i++)
		indexMaxHeap.insert(i, arr[i]);
	assert(indexMaxHeap.testIndexes());

	for (int i = n - 1; i >= 0; i--)
		arr[i] = indexMaxHeap.extractMax();
}

int main() {

	int n = 1000000;

	int* arr = SortTestHelper::generateRandomArray(n, 0, n);
	SortTestHelper::testSort("Heap Sort Using Index-Max-Heap", heapSortUsingIndexMaxHeap, arr, n);
	delete[] arr;

	return 0;
}