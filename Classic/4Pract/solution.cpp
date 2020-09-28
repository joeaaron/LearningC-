#include "BigInteger.h"
#include "strings.h"
//TEST BIGINTEGER
//int main()
//{
//	string s[2];
//	cin >> s[0] >> s[1];
//	BigInteger a[2];
//	for (int i = 0; i < 2; i++) {
//		a[i] = s[i];
//	}
//	BigInteger c = a[0] * a[1];
//	for (int i = c.s.size() - 1; i >= 0; i--) {
//		cout << c.s[i];
//	}
//	cout << endl;
//	system("pause");
//	return 0;
//
//}

//TEST STRINGS
int main()
{
	String str1("Hello");
	cout << str1.data() << '\n';
	cout << str1.length() << '\n';
	cout << "----------\n";
	String str2 = "Word";
	cout << str2 << '\n';
	cout << "----------\n";
	String str3 = str1 + str2;
	cout << str3.data() << '\n';
	cout << str3.length() << '\n';
	system("pause");
	return 0;
}