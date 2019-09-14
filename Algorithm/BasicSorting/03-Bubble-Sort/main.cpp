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
		// 优化, 每一趟Bubble Sort都将最大的元素放在了最后的位置
	    // 所以下一次排序, 最后的元素可以不再考虑
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
				// 记录最后一次的交换位置,在此之后的元素在下一轮扫描中均不考虑
				newn = i;
			}
		}
		n = newn;
	} while (newn > 0);

}

int main()
{
	// 测试模板函数，传入整型数组
	int a[10] = { 3, 9, 18, 7, 34, 5, 14, 2, 66, 1 };
	bubbleSort2(a, 10);
	for (int i = 0; i < 10; i++)
		cout << a[i] << " ";
	cout << endl;
}