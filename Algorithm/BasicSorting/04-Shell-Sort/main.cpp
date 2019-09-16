#include <iostream>
#include <algorithm>
#include "SortTestHelper.h"

using namespace std;

template<typename T>
void shellSort(T arr[], int n)
{
	int h = 1;
	while (h < n / 3)
		h = 3 * h + 1;
	while (h>=1)
	{	
		for (int i = h; i < n; i++)
		{
			//��arr[i], arr[i-h], arr[i-3*h]...ʹ�ò�������
			T e = arr[i];
			int j;
			for (j = i; j >= h && e < arr[j - h]; j -= h)
				arr[j] = arr[j - h];
			arr[j] = e;
		}
		h /= 3;
	}
}


int main()
{
	// ����ģ�庯����������������
	int a[10] = { 3, 9, 18, 7, 34, 5, 14, 2, 66, 1 };
	shellSort(a, 10);
	for (int i = 0; i < 10; i++)
		cout << a[i] << " ";
	cout << endl;
}