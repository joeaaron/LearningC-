#include <iostream>

#include <boost/any.hpp>
#include <boost/format.hpp>
#include <boost/assert.hpp>
#include <boost/shared_ptr.hpp>


#define PRINT(xxx) ( std::cout << boost::format("%-20s = ") % (#xxx) << (xxx) << std::endl)

//���һ��any�Ƿ���ת����ĳ�����ͣ� RTTI
template<typename T>
bool can_cast_to(const boost::any& a){
	return a.type() == typeid(T);
}

//��ȡany�ڲ���������ã����������޸Ķ����ֵ
template<typename T>
T& get_ref(boost::any& a){
	BOOST_ASSERT_MSG(can_cast_to<T>(a), "any����ת�����󣡣���");
	return boost::any_cast<T&>(a);
}

//��ȡԭ�����ָ��
template<typename T>
T* get_ptr(boost::any& a){
	BOOST_ASSERT_MSG(can_cast_to<T>(a), "any����ת�����󣡣���");
	return boost::any_cast<T>(&a);
}

int main()
{
	/*1.�����÷�*/
	puts("1.�����÷�");
	boost::any a(1);
	//��ȡԭ����
	PRINT(boost::any_cast<int>(a));
	
	//�޸�ԭ�����ֵ����Ϊ��ֵ���ü���
	boost::any_cast<int&>(a) = 100;
	PRINT(boost::any_cast<int>(a));

	//��ȡԭ�����ָ��
	int* ss = boost::any_cast<int>(&a);
	PRINT(*ss);

	//��ȡԭ�����������Ϣ
	PRINT(a.type().name());

	//���any���Ƿ񱣴��˶���
	std::cout << std::boolalpha;        //������ӡ��false��true�ĺ���
	PRINT(a.empty());
	puts("");

	/**************************************************************************/
	/*2.����ָ�룬������any������ϵ�ԭʼ��ָ�룬������ڴ�й¶,Ӧʹ������ָ��*/
	puts("2.����ָ��");
	int* p = new int(2);
	boost::any ptr_any(p);
	int* tmp = boost::any_cast<int*>(ptr_any);
	PRINT(*tmp);
	//Ӧ��ʹ������ָ�룬����ֻ��ʹ��shared_ptr,��Ϊscoped_ptr���ܱ�����
	//������any����ʱ�������shared_ptr�������������ͷ�����е���Դ
	boost::any shared_any(boost::shared_ptr<int>(new int(3)));
	auto p_shared = boost::any_cast<boost::shared_ptr<int>>(shared_any);
	PRINT(*p_shared);
	puts("");

	/**************************************************************************/
	/*3.��������*/
	puts("3.��������");
	std::string str("hello");
	boost::any xxx(str);

	//���һ��any�Ƿ���ת����ĳ�����ͣ�RTTI
	PRINT(can_cast_to<int>(xxx));
	PRINT(can_cast_to<std::string>(xxx));

	//��ȡany�ڲ���������ã����������޸Ķ����ֵ
	get_ref<std::string>(xxx) = "world!";
	//��ȡԭ�����ָ��
	PRINT(get_ptr<std::string>(xxx)->size());
	puts("");
	/**************************************************************************/
	/*4.��������*/
	puts("4.��������");
	std::vector<boost::any> vec{
		1,
		std::string("��ã�"),
		1.414,
		boost::shared_ptr<std::string>(new std::string("end"))
	};
	PRINT(boost::any_cast<int>(vec[0]));
	PRINT(boost::any_cast<std::string&>(vec[1]));
	PRINT(boost::any_cast<double>(vec[2]));
	PRINT(*boost::any_cast<boost::shared_ptr<std::string>>(vec[3]));

	system("pause");
	return 0;
}
