#include <iostream>
using namespace std;

class Figure{
protected:
	double x, y;
public:
	void set(double i, double j)
	{
		x = i;
		y = j;
	}
	virtual void area() = 0;
};

class Triangle :public Figure{
public:
	void area(){
		cout << "三角形面积：" << x*y*0.5 << endl;
	}
};

class Rectangle :public Figure{
public:
	void area(){
		cout << "这是矩形，它的面积是：" << x*y << endl;
	}
};

int main()
{
	//定义抽象类指针
	Figure *pF = NULL;

	Rectangle r;
	Triangle t;
	t.set(10, 20);
	pF = &t;
	pF->area();
	r.set(10, 20);
	pF = &r;
	pF->area();
	//定义抽象类引用
	Figure &rF = t;
	rF.set(20, 20);
	rF.area();
	system("pause");
	return 0;

}