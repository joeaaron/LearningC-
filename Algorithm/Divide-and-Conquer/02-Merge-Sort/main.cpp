/*
********简单归并排序**********
*/
#include <iostream>
#include <algorithm>

using namespace std;

void Merge(int A[], int low, int mid, int high)
{
	int *B = new int[high - low + 1];		// 申请一个辅助数组
	int i = low, j = mid + 1, k = 0;
	while (i <= mid && j <= high)
	{
		if (A[i] <= A[j])
			B[k++] = A[i++];
		else
			B[k++] = A[j++];
	}
	while (i <= mid) B[k++] = A[i++];		// 将数组中剩下的元素赋值到数组B中
	while (j <= high) B[k++] = A[j++];
	for (i = low, k = 0; i <= high; i++)
		A[i] = B[k++];
}

void MergeSort(int A[], int low, int high)
{
	if (low < high)
	{
		int mid = (low + high) / 2;
		MergeSort(A, low, mid);
		MergeSort(A, mid + 1, high);
		Merge(A, low, mid, high);
	}
}

int main()
{
	int n, A[100];
	cout << "请输入数列中的元素个数n为：" << endl;
	cin >> n;
	cout << "请依次输入数列中的元素： " << endl;
	for (int i = 0; i < n;i++)
	{
		cin >> A[i];
	}
	MergeSort(A, 0, n - 1);
	cout << "合并排序结果：" << endl;
	for (int i = 0; i < n;i++)
	{
		cout << A[i] << " ";
	}
	cout << endl;
	system("pause");
	return 0;
}

/*
*********************************************************************
*/

