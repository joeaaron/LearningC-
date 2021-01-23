/*
********简单快速排序**********
*/
#include <iostream>
#include <algorithm>

using namespace std;

int Partition(int r[], int row, int high)
{
	int i = row, j = high, pivot = r[row];				
	while (i<j)
	{
		while (i < j && r[j] > pivot)	
			j--;				// 向左扫描
		if (i < j)	swap(r[i++], r[j]);
		while (i < j && r[i] <= pivot)	
			i++;				// 向右扫描
		if (i < j) swap(r[i], r[j--]);
	}
	return i;					// 返回最终划分完成后基准元素所在的位置
}

void QuickSort(int R[], int row, int high)
{
	int mid;
	if (row < high)
	{
		mid = Partition(R, row, high);
		QuickSort(R, row, mid - 1);
		QuickSort(R, mid + 1, high);
	}
}
int main()
{
	int a[1000];
	int i, N;
	cout << "请先输入要排序的数据的个数： ";
	cin >> N;
	cout << "请输入要排序的数据： ";
	for (i = 0; i < N; i++)
		cin >> a[i];
	cout << endl;
	QuickSort(a, 0, N - 1);
	cout << "排序后的序列为： "<< endl;
	for (i = 0; i < N; i++)
		cout << a[i] << " ";

	cout << endl;
	system("pause");
	return 0;
}

/*
*********************************************************************
*/

//#include <iostream>
//
//int main(int argc, char *argv[])
//{
//	int i = 30 - 1;
//	while (true)
//	{
//		if (i % 7 == 0)
//			break;
//		else
//			i += 30;
//	}
//
//	printf("result is: %d\n", i);
//	return 0;
//}