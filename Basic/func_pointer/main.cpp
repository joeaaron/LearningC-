#include <iostream>
#include <map>


// ����һ������ָ��
class Test;
typedef void(Test::*pFun)(int nCode);

// һ��������
class Test
{
public:
	enum MyCode
	{
		code_1,
		code_2 = 3,
		code_3,
		code_4,
		code_5,
		code_6,
		code_7
	};

	// ��ʼ����������봦�����Ķ�Ӧ��ϵ
	void InitHandlerMap();

	void Process(int nCode);


	void			Handler1(int nCode){ printf("code = %d\n", nCode); }
	void			Handler2(int nCode){ printf("code = %d\n", nCode); }
	void			Handler3(int nCode){ printf("code = %d\n", nCode); }
	void			Handler4(int nCode){ printf("code = %d\n", nCode); }
	void			Handlern(int nCode){ printf("code = %d\n", nCode); }


private:
	std::map<int, pFun>			m_pHandlerMap;

};


void Test::InitHandlerMap()
{
	m_pHandlerMap.insert(std::make_pair(Test::code_1, &Test::Handler1));
	m_pHandlerMap.insert(std::make_pair(Test::code_2, &Test::Handler2));
	m_pHandlerMap.insert(std::make_pair(Test::code_3, &Test::Handler3));
	m_pHandlerMap.insert(std::make_pair(Test::code_4, &Test::Handler4));
}

void Test::Process(int code)
{
	std::map<int, pFun>::iterator it;
	for (it = m_pHandlerMap.begin(); it != m_pHandlerMap.end(); ++it)
	{
		if (it->first == code)
		{
			(this->*(it->second))(code);
			break;
		}
	}
}

int main()
{
	int code = Test::code_2;

	Test test;

	test.InitHandlerMap();

	test.Process(code);

	system("pause");

	return 0;
}
