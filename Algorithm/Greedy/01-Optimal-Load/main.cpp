/*
********最优装载问题**********
*/
#include <iostream>
#include <algorithm>

using namespace std;

const int N = 1000005;
double w[N];		// 古董的重量数组

int main()
{
	double c;
	int n;
	cout << "请输入载重量c及古董个数n:" << endl;
	cin >> c >> n;
	cout << "请输入每个古董的重量，用空格分开： " << endl;
	for (int i = 0; i < n;i++)
	{
		cin >> w[i];		// 输入每个物品重量
	}
	sort(w, w + n);					// 按古董重量升序排序
	//double tmp = 0.0;
	//int ans = 0;					// tmp为已装载到船上的古董重量，ans为已装载的古董个数
	//if (tmp <= c)
	//	ans++;
	//else
	//	break;

	double tmp = 0, ans = n;
	for (int i = 0; i < n;i++)
	{
		tmp += w[i];
		if (tmp >= c)
		{
			if (tmp == c)
				ans = i + 1;
			else ans = i;
			break;
		}
	}

	cout << "能装入的古董最大数量为Ans=";
	cout << ans << endl;

	system("pause");
	return 0;
}

/*
**************1.为什么在没有装满的情况下，仍然是最优解？************************
**************2.如果想知道装了什么古董，还需要添加什么程序？********************
*/

