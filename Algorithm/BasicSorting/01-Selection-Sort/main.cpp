#include <iostream>
#include <algorithm>
#include "Student.h"
#include "SortTestHelper.h"

using namespace std;

template<typename T>
void selectionSort(T arr[], int n)
{
	for(int i = 0; i < n; i++)
	{
		//Ѱ��[i, n)���������Сֵ
		int minIndex = i;
		for(int j = i + 1;j < n; j++)
			if(arr[j] < arr[minIndex])
				minIndex = j;
			
		swap(arr[i], arr[minIndex]);
	}
		

}

// ��лgithub @zhengquan45 ���, ���Զ�ѡ����������Ż�
// ��ÿһ����, ����ͬʱ�ҵ���ǰδ����Ԫ�ص����ֵ����Сֵ
template<typename T>
void selectionSortNew(T arr[], int n)
{
	int left = 0, right = n - 1;
	while (left < right){
		int minIndex = left;
		int maxIndex = right;

		//ÿһ�ֲ���ʱ��Ҫ��֤arr[minIndex] <= arr[maxIndex]
		if (arr[minIndex] > arr[maxIndex]){
			swap(arr[minIndex], arr[maxIndex]);
		}

		for (int i = left + 1; i < right; i++){
			if (arr[i] < arr[minIndex])
				minIndex = i;
			else if (arr[i] > arr[minIndex])
				minIndex = i;
		}

		swap(arr[left], arr[minIndex]);
		swap(arr[right], arr[maxIndex]);

		left ++;
		right --;
	}

	return;
}

int main() {
	/*
	// ����ģ�庯����������������
	int a[10] = { 10, 9, 8, 7, 6, 5, 4, 3, 2, 1 };
	selectionSort(a, 10);
	for (int i = 0; i < 10; i++)
	cout << a[i] << " ";
	cout << endl;

	// ����ģ�庯�������븡��������
	float b[4] = { 4.4, 3.3, 2.2, 1.1 };
	selectionSort(b, 4);
	for (int i = 0; i < 4; i++)
	cout << b[i] << " ";
	cout << endl;

	// ����ģ�庯���������ַ�������
	string c[4] = { "D", "C", "B", "A" };
	selectionSort(c, 4);
	for (int i = 0; i < 4; i++)
	cout << c[i] << " ";
	cout << endl;

	// ����ģ�庯���������Զ���ṹ��Student����
	Student d[4] = { { "D", 90 }, { "C", 100 }, { "B", 95 }, { "A", 95 } };
	selectionSort(d, 4);
	for (int i = 0; i < 4; i++)
	cout << d[i];
	cout << endl;
	*/

	/*
	// ���������㷨��������
	int N = 20000;
	int *arr = SortTestHelper::generateRandomArray(N, 0, 100000);
	selectionSort(arr, N);
	SortTestHelper::printArray(arr, N);
	delete[] arr;
	*/

	int n = 20000;
	int *arr = SortTestHelper::generateRandomArray(n, 0, n);
	SortTestHelper::testSort("Selection Sort", selectionSort, arr, n);
	delete[] arr;

	return 0;
}