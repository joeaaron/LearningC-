#include <iostream>

using namespace std;

// º¯ÊıÄ£°å
template<class T>
T CompareMax(T t1, T t2)
{
	return t1 > t2 ? t1 : t2;
}

template<> char* CompareMax<char *>(char *s1, char *s2)
{
	return strcmp(s1, s2) >= 0 ? s1 : s2;
}

int main()
{
	cout << CompareMax(1, 2) << endl;
	cout << CompareMax("asda", "qweq") << endl;
	system("pause");
	return 0;
}