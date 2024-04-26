#include <iostream>
#include <functional>
using namespace std;

/*�ص�����ԭ������*/
typedef function<void(int)> CALLBACK;

/*���SDK�ײ�A��*/
class A_Camera
{
public:
	void regeditCallBack(CALLBACK fun)/*ע��ص�����*/
	{
		_fun = fun;
	}

	void getFrame()/*�ڲ���ȡͼ������B������߲���Ҫ������ʲôʱ���ִ�У�*/
	{
		/*�ɼ���һ֡����_frame*/
		/****�ڲ�����***/
		/***�ڲ�����***/

		_frame = rand() % 10;
		_fun(_frame);/*�ش���B_My��*/
	}

private:
	int _frame;
	CALLBACK _fun;
};

/*Ӧ�ò�B��*/
class B_My
{
public:
	void callBackFun(int frame)/*��ȡ��A���ͼ�񣬴�ʱframe����һ֡����*/
	{
		cout << "B���ȡ��һ֡���ݣ�" << frame << endl;
	}
};

int main(int argc, char** argv)
{
	/*����Ӧ�ò�B�����*/
	B_My B;

	auto Fun = bind(&B_My::callBackFun, B, placeholders::_1);/*��תһ��,����C++11����*/

	/*�����ײ����A��*/
	A_Camera camera;
	camera.regeditCallBack(Fun);/*��B��ķ���ע���A��*/

	/*����ֻ��ģ��A���ڲ�������ȡ��ͼƬ��һ��ģ�ⴥ��10��*/
	for (int i = 0; i < 10; ++i)
	{
		camera.getFrame();
	}

	return 0;
}

