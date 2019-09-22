#include <iostream>
#include <algorithm>
#include <ctime>
#include "SortTestHelper.h"
#include "MergeSort.h"
#include "InsertionSort.h"
#include "QuickSort.h"

using namespace std;


// �ݹ����·���������㷨
template <typename T>
void __quickSort3Ways(T arr[], int l, int r) {

	// ����С��ģ����, ʹ�ò�����������Ż�
	if (r - l <= 15) {
		insertionSort(arr, l, r);
		return;
	}

	// �����arr[l...r]�ķ�Χ��, ѡ��һ����ֵ��Ϊ�궨��pivot
	swap(arr[l], arr[rand() % (r - l + 1) + l]);

	T v = arr[l];

	int lt = l;     // arr[l+1...lt] < v
	int gt = r + 1; // arr[gt...r] > v
	int i = l + 1;    // arr[lt+1...i) == v
	while (i < gt) {
		if (arr[i] < v) {
			swap(arr[i], arr[lt + 1]);
			i++;
			lt++;
		}
		else if (arr[i] > v) {
			swap(arr[i], arr[gt - 1]);
			gt--;
		}
		else { // arr[i] == v
			i++;
		}
	}

	swap(arr[l], arr[lt]);

	__quickSort3Ways(arr, l, lt - 1);
	__quickSort3Ways(arr, gt, r);
}

template <typename T>
void quickSort3Ways(T arr[], int n) {

	srand(time(NULL));
	__quickSort3Ways(arr, 0, n - 1);
}


// �Ƚ�Merge Sort��˫·�����������·�������������㷨������Ч��
// ���ڰ����д����ظ����ݵ�����, ��·�����о޴������
// ����һ���Ե��������ͽ������������, ��·���ŵ�Ч����Ȼ�������ŵ�, �������ڷǳ����Խ��ܵķ�Χ��
// ���, ��һЩ������, ��·������Ĭ�ϵ����Կ⺯����ʹ�õ������㷨������Java:)
int main() {

	int n = 1000000;

	// ����1 һ���Բ���
	cout << "Test for random array, size = " << n << ", random range [0, " << n << "]" << endl;
	int* arr1 = SortTestHelper::generateRandomArray(n, 0, n);
	int* arr2 = SortTestHelper::copyIntArray(arr1, n);
	int* arr3 = SortTestHelper::copyIntArray(arr1, n);

	SortTestHelper::testSort("Merge Sort", mergeSort, arr1, n);
	SortTestHelper::testSort("Quick Sort", quickSort, arr2, n);
	SortTestHelper::testSort("Quick Sort 3 Ways", quickSort3Ways, arr3, n);

	delete[] arr1;
	delete[] arr2;
	delete[] arr3;

	cout << endl;


	// ����2 ���Խ������������
	int swapTimes = 100;
	cout << "Test for nearly ordered array, size = " << n << ", swap time = " << swapTimes << endl;
	arr1 = SortTestHelper::generateNearlyOrderedArray(n, swapTimes);
	arr2 = SortTestHelper::copyIntArray(arr1, n);
	arr3 = SortTestHelper::copyIntArray(arr1, n);

	SortTestHelper::testSort("Merge Sort", mergeSort, arr1, n);
	SortTestHelper::testSort("Quick Sort", quickSort, arr2, n);
	SortTestHelper::testSort("Quick Sort 3 Ways", quickSort3Ways, arr3, n);

	delete[] arr1;
	delete[] arr2;
	delete[] arr3;

	cout << endl;


	// ����3 ���Դ��ڰ���������ͬԪ�ص�����
	cout << "Test for random array, size = " << n << ", random range [0,10]" << endl;
	arr1 = SortTestHelper::generateRandomArray(n, 0, 10);
	arr2 = SortTestHelper::copyIntArray(arr1, n);
	arr3 = SortTestHelper::copyIntArray(arr1, n);

	SortTestHelper::testSort("Merge Sort", mergeSort, arr1, n);
	SortTestHelper::testSort("Quick Sort", quickSort, arr2, n);
	SortTestHelper::testSort("Quick Sort 3 Ways", quickSort3Ways, arr3, n);

	delete[] arr1;
	delete[] arr2;
	delete[] arr3;


	return 0;
}