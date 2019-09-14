#include <iostream>
#include <algorithm>
#include "SortTestHelper.h"

using namespace std;

template<typename T>
void bubbleSort(T arr[], int n)
{
	bool swapped;
	do {
		swapped = false;
		for (int i = 1; i < n; i++) {
			if (arr[i-1] > arr[i])
			{
				swap(arr[i - 1], arr[i]);
				swapped = true;

			}
		}
		// �Ż�, ÿһ��Bubble Sort��������Ԫ�ط���������λ��
	    // ������һ������, ����Ԫ�ؿ��Բ��ٿ���
		n--;
	} while (swapped);
}

template<typename T>
void bubbleSort2(T arr[], int n)
{
	int newn;

	do {
		newn = 0;
		for (int i = 1; i < n; i++) {
			if (arr[i - 1] > arr[i]) {
				swap(arr[i - 1], arr[i]);
				// ��¼���һ�εĽ���λ��,�ڴ�֮���Ԫ������һ��ɨ���о�������
				newn = i;
			}
		}
		n = newn;
	} while (newn > 0);

}

int main()
{
	// ����ģ�庯����������������
	int a[10] = { 3, 9, 18, 7, 34, 5, 14, 2, 66, 1 };
	bubbleSort2(a, 10);
	for (int i = 0; i < 10; i++)
		cout << a[i] << " ";
	cout << endl;
}