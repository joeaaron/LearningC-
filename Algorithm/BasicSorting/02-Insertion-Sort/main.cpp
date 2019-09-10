#include <iostream>
#include <algorithm>
#include "SortTestHelper.h"
#include "SelectionSort.h"

using namespace std;

template<typename T>
void insertionSort(T arr[], int n){

	for (int i = 1; i < n; i++) {

		// Ѱ��Ԫ��arr[i]���ʵĲ���λ��
		// д��1
		//        for( int j = i ; j > 0 ; j-- )
		//            if( arr[j] < arr[j-1] )
		//                swap( arr[j] , arr[j-1] );
		//            else
		//                break;

		// д��2
		for (int j = i; j > 0 && arr[j] < arr[j - 1]; j--)
			swap(arr[j], arr[j - 1]);

	}

	return;
}

// �Ƚ�SelectionSort��InsertionSort���������㷨������Ч��
// ��ʱ�� ���������ѡ�����������Ե�
int main() {

	int n = 20000;

	cout << "Test for random array, size = " << n << ", random range [0, " << n << "]" << endl;
	int *arr1 = SortTestHelper::generateRandomArray(n, 0, n);
	int *arr2 = SortTestHelper::copyIntArray(arr1, n);

	SortTestHelper::testSort("Insertion Sort", insertionSort, arr1, n);
	SortTestHelper::testSort("Selection Sort", selectionSort, arr2, n);

	delete[] arr1;
	delete[] arr2;

	cout << endl;

	return 0;
}