/*
********猜数游戏**********
*/
#include <iostream>
#include <algorithm>

using namespace std;

const int M = 10000;
int x, n, i;
int s[M];
int BinarySearch(int n, int s[], int x)
{
	int low = 0, high = n - 1;			// low指向数组的第一个元素，high指向数组的最后一个元素
	while (low < high)
	{
		int mid = (low + high) / 2;
		if (x == s[mid])
			return mid;
		else if (x < s[mid])
			high = mid - 1;
		else
			low = mid + 1;
	}
	return -1;
}

int main()
{
	cout << "请输入数列中元素的个数n为：";
	while (cin>>n)
	{
		cout << "请依次输入数列中元素：";
		for (i = 0; i < n; i++)
			cin >> s[i];
		sort(s, s + n);
		cout << "排序后的数组为： ";
		for (size_t j = 0; j < n; j++)
			cout << s[j] << " ";
		cout << endl;
		cout << "请输入要查找的元素： ";
		cin >> x;
		i = BinarySearch(n, s, x);
		if (1 == i)
			cout << "该数列中没有要查找的元素" << endl;
		else
			cout << "要查找的元素在第" << i + 1 << "位" << endl;

	}

	system("pause");
	return 0;
}

/*
**************1.能否通过递归来实现？*********************
*/

