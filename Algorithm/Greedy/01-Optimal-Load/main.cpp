/*
********����װ������**********
*/
#include <iostream>
#include <algorithm>

using namespace std;

const int N = 1000005;
double w[N];		// �Ŷ�����������

int main()
{
	double c;
	int n;
	cout << "������������c���Ŷ�����n:" << endl;
	cin >> c >> n;
	cout << "������ÿ���Ŷ����������ÿո�ֿ��� " << endl;
	for (int i = 0; i < n;i++)
	{
		cin >> w[i];		// ����ÿ����Ʒ����
	}
	sort(w, w + n);					// ���Ŷ�������������
	//double tmp = 0.0;
	//int ans = 0;					// tmpΪ��װ�ص����ϵĹŶ�������ansΪ��װ�صĹŶ�����
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

	cout << "��װ��ĹŶ��������ΪAns=";
	cout << ans << endl;

	system("pause");
	return 0;
}

/*
**************1.Ϊʲô��û��װ��������£���Ȼ�����Ž⣿************************
**************2.�����֪��װ��ʲô�Ŷ�������Ҫ���ʲô����********************
*/

