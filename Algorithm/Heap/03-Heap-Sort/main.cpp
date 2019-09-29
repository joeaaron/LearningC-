#include <iostream>
#include <algorithm>
#include "Heap.h"

using namespace std;
// ԭʼ��shiftDown����
template<typename T>
void __shiftDown(T arr[], int n, int k) {

	while (2 * k + 1 < n) {
		int j = 2 * k + 1;
		if (j + 1 < n && arr[j + 1] > arr[j])
			j += 1;

		if (arr[k] >= arr[j])break;

		swap(arr[k], arr[j]);
		k = j;
	}
}

// �Ż���shiftDown����, ʹ�ø�ֵ�ķ�ʽȡ�����ϵ�swap,
// ���Ż�˼�������֮ǰ�Բ�����������Ż���˼·��һ�µ�
template<typename T>
void __shiftDown2(T arr[], int n, int k) {

	T e = arr[k];
	while (2 * k + 1 < n) {
		int j = 2 * k + 1;
		if (j + 1 < n && arr[j + 1] > arr[j])
			j += 1;

		if (e >= arr[j]) break;

		arr[k] = arr[j];
		k = j;
	}

	arr[k] = e;
}

template<typename T>
void heapSort(T arr[], int n){

	for (int i = (n - 1 - 1) / 2; i >= 0; i--)
		__shiftDown2(arr, n, i);

	for (int i = n - 1; i > 0; i--) {
		swap(arr[0], arr[i]);
		__shiftDown2(arr, i, 0);
	}

}

int main()
{
	// ����ģ�庯����������������
	int a[10] = { 3, 9, 18, 7, 34, 5, 14, 2, 66, 1 };
	heapSort(a, 10);
	for (int i = 0; i < 10; i++)
		cout << a[i] << " ";
	cout << endl;
}