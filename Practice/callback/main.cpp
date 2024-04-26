#include <iostream>
#include <functional>
using namespace std;

/*回调函数原型声明*/
typedef function<void(int)> CALLBACK;

/*相机SDK底层A类*/
class A_Camera
{
public:
	void regeditCallBack(CALLBACK fun)/*注册回调函数*/
	{
		_fun = fun;
	}

	void getFrame()/*内部获取图像函数（B类调用者不需要关心它什么时候会执行）*/
	{
		/*采集到一帧数据_frame*/
		/****内部操作***/
		/***内部操作***/

		_frame = rand() % 10;
		_fun(_frame);/*回传给B_My类*/
	}

private:
	int _frame;
	CALLBACK _fun;
};

/*应用层B类*/
class B_My
{
public:
	void callBackFun(int frame)/*获取到A类的图像，此时frame就是一帧数据*/
	{
		cout << "B类获取到一帧数据：" << frame << endl;
	}
};

int main(int argc, char** argv)
{
	/*声明应用层B类对象*/
	B_My B;

	auto Fun = bind(&B_My::callBackFun, B, placeholders::_1);/*中转一下,利用C++11特性*/

	/*声明底层相机A类*/
	A_Camera camera;
	camera.regeditCallBack(Fun);/*把B类的方法注册给A类*/

	/*以下只是模拟A类内部触发获取到图片，一共模拟触发10次*/
	for (int i = 0; i < 10; ++i)
	{
		camera.getFrame();
	}

	return 0;
}

