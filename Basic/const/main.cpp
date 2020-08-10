/*
const对象只能访问const成员函数,而非const对象可以访问任意的成员函数,包括const成员函数.
*/
#include<iostream>
#include"apple.cpp"
using namespace std;

Apple::Apple(int i) :apple_number(i)
{

}
void Apple::add(int num)
{
	take(num);
	
}
void Apple::add(int num) const
{
	take(num);

}
void Apple::take(int num) const
{
	cout << "take func " << num << endl;
}

int Apple::getCount() const
{
	take(1);
	//add(); //error
	return apple_number;
}
int main()
{
	Apple a(2);
	cout << a.getCount() << endl;
	a.add(10);
	const Apple b(3);
	b.add(100);
	system("pause");
	return 0;
}
