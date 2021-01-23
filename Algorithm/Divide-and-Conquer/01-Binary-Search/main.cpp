/*
********������Ϸ**********
*/
#include <iostream>
#include <algorithm>

using namespace std;

const int M = 10000;
int x, n, i;
int s[M];
int BinarySearch(int n, int s[], int x)
{
	int low = 0, high = n - 1;			// lowָ������ĵ�һ��Ԫ�أ�highָ����������һ��Ԫ��
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
	cout << "������������Ԫ�صĸ���nΪ��";
	while (cin>>n)
	{
		cout << "����������������Ԫ�أ�";
		for (i = 0; i < n; i++)
			cin >> s[i];
		sort(s, s + n);
		cout << "����������Ϊ�� ";
		for (size_t j = 0; j < n; j++)
			cout << s[j] << " ";
		cout << endl;
		cout << "������Ҫ���ҵ�Ԫ�أ� ";
		cin >> x;
		i = BinarySearch(n, s, x);
		if (1 == i)
			cout << "��������û��Ҫ���ҵ�Ԫ��" << endl;
		else
			cout << "Ҫ���ҵ�Ԫ���ڵ�" << i + 1 << "λ" << endl;

	}

	system("pause");
	return 0;
}

/*
**************1.�ܷ�ͨ���ݹ���ʵ�֣�*********************
*/

