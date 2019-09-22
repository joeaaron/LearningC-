#include <iostream>
#include <algorithm>
#include "SortTestHelper.h"
#include "MergeSort.h"
#include "InsertionSort.h"

using namespace std;
// ��arr[l...r]���ֽ���partition����
// ����p, ʹ��arr[l...p-1] < arr[p] ; arr[p+1...r] > arr[p]
template <typename T>
int __partition(T arr[], int l, int r) {

	T v = arr[l];

	int j = l; // arr[l+1...j] < v ; arr[j+1...i) > v
	for (int i = l + 1; i <= r; i++)
		if (arr[i] < v) {
			j++;
			swap(arr[j], arr[i]);
		}

	swap(arr[l], arr[j]);

	return j;
}

// ˫·���������partition
// ����p, ʹ��arr[l...p-1] <= arr[p] ; arr[p+1...r] >= arr[p]
// ˫·���Ŵ����Ԫ�����õ���arr[p]��ʱ��Ҫע�⣬��������ע�ͣ���
template <typename T>
int _partition2(T arr[], int l, int r) {

	// �����arr[l...r]�ķ�Χ��, ѡ��һ����ֵ��Ϊ�궨��pivot
	swap(arr[l], arr[rand() % (r - l + 1) + l]);
	T v = arr[l];

	// arr[l+1...i) <= v; arr(j...r] >= v
	int i = l + 1, j = r;
	while (true) {
		// ע������ı߽�, arr[i] < v, ������arr[i] <= v
		// ˼��һ��Ϊʲô?
		while (i <= r && arr[i] < v)
			i++;

		// ע������ı߽�, arr[j] > v, ������arr[j] >= v
		// ˼��һ��Ϊʲô?
		while (j >= l + 1 && arr[j] > v)
			j--;

		// ��������������߽���趨, �е�ͬѧ�ڿγ̵��ʴ����кܺõĻش�:)
		// ��ҿ��Բο�: http://coding.imooc.com/learn/questiondetail/4920.html

		if (i > j)
			break;

		swap(arr[i], arr[j]);
		i++;
		j--;
	}

	swap(arr[l], arr[j]);

	return j;
}

// ��arr[l...r]���ֽ��п�������
template <typename T>
void _quickSort(T arr[], int l, int r) {

	// ����С��ģ����, ʹ�ò�����������Ż�
	if (r - l <= 15) {
		insertionSort(arr, l, r);
		return;
	}

	// ����˫·���������partition
	int p = _partition2(arr, l, r);
	_quickSort(arr, l, p - 1);
	_quickSort(arr, p + 1, r);
}

template <typename T>
void quickSort(T arr[], int n) {

	srand(time(NULL));
	_quickSort(arr, 0, n - 1);
}


// �Ƚ�Merge Sort��˫·�����������������㷨������Ч��
int main() {

	int n = 1000000;

	// ����1 һ���Բ���
	cout << "Test for random array, size = " << n << ", random range [0, " << n << "]" << endl;
	int* arr1 = SortTestHelper::generateRandomArray(n, 0, n);
	int* arr2 = SortTestHelper::copyIntArray(arr1, n);

	SortTestHelper::testSort("Merge Sort", mergeSort, arr1, n);
	SortTestHelper::testSort("Quick Sort", quickSort, arr2, n);

	delete[] arr1;
	delete[] arr2;

	cout << endl;


	// ����2 ���Խ������������
	// ˫·���������㷨Ҳ�������ɴ���������������
	int swapTimes = 100;
	cout << "Test for nearly ordered array, size = " << n << ", swap time = " << swapTimes << endl;
	arr1 = SortTestHelper::generateNearlyOrderedArray(n, swapTimes);
	arr2 = SortTestHelper::copyIntArray(arr1, n);

	SortTestHelper::testSort("Merge Sort", mergeSort, arr1, n);
	SortTestHelper::testSort("Quick Sort", quickSort, arr2, n);

	delete[] arr1;
	delete[] arr2;

	cout << endl;


	// ����3 ���Դ��ڰ���������ͬԪ�ص�����
	// ʹ��˫���������, ���ǵĿ��������㷨�������ɵĴ����������Ԫ�ص�����
	cout << "Test for random array, size = " << n << ", random range [0,10]" << endl;
	arr1 = SortTestHelper::generateRandomArray(n, 0, 10);
	arr2 = SortTestHelper::copyIntArray(arr1, n);

	SortTestHelper::testSort("Merge Sort", mergeSort, arr1, n);
	SortTestHelper::testSort("Quick Sort", quickSort, arr2, n);

	delete[] arr1;
	delete[] arr2;


	return 0;
}