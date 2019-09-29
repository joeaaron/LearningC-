#include <iostream>
#include <algorithm>
#include "Heap.h"

using namespace std;
// 原始的shiftDown过程
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

// 优化的shiftDown过程, 使用赋值的方式取代不断的swap,
// 该优化思想和我们之前对插入排序进行优化的思路是一致的
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
	// 测试模板函数，传入整型数组
	int a[10] = { 3, 9, 18, 7, 34, 5, 14, 2, 66, 1 };
	heapSort(a, 10);
	for (int i = 0; i < 10; i++)
		cout << a[i] << " ";
	cout << endl;
}