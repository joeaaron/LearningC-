#include "BigInteger.h"
#include "strings.h"
#include "vectors.h"
//TEST BIGINTEGER
void TEST1()
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

}

//TEST STRINGS
void TEST2()
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

}

//TEST VECTORS
void TEST3()
{
	Vector<int> v(3, 1);
	v.print();
	cout << "size: " << v.size << " capacity: " << v.capacity << endl;
	for (int iter = 0; iter < 11; iter++)
	{
		v.push_back(iter);
	}
	v.print();
	v.pop_back();
	v.pop_back();
	v.pop_back();
	v.print();
	v[3] = 10086;
	v.print();

	cout << "what" << endl;
	Vector<int> v2(v);
	v2.print();

	Vector<int> v3 = v;
	v3.print();

	Vector<int> v4;
	v4 = v;
	v4.print();

	cout << "size: " << v4.size << " capacity: " << v4.capacity << endl;
	v4.insert(1, 11111);
	v4.insert(0, 22222);
	v4.insert(v4.size - 1, 33333);
	v4.insert(v4.size, 44444);
	v4.print();
	v4.clear();
	v4.print();

}

int main()
{
	TEST3();
	system("pause");
	return 0;
}