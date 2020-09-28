#include "BigInteger.h"

int main()
{
	string s[2];
	cin >> s[0] >> s[1];
	BigInteger a[2];
	for (int i = 0; i < 2; i++) {
		a[i] = s[i];
	}
	BigInteger c = a[0] * a[1];
	for (int i = c.s.size() - 1; i >= 0; i--) {
		cout << c.s[i];
	}
	cout << endl;
	system("pause");
	return 0;

}