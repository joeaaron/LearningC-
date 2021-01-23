#include "strings.h"

using std::cout;
using std::ostream;
using std::istream;

void String::init(const char* str)
{
	if (nullptr == str){
		_length = 0;
		buffer = nullptr;
	}
	else{
		_length = std::strlen(str);
		buffer = new char[_length + 1];
		std::strcpy(buffer, str);
	}
}

size_t String::length()
{
	if (0 == _length) _length = std::strlen(buffer);
	return _length;
}

const char* String::data()
{
	return buffer;
}

String::String(const char* str) {
	init(str);
	cout << "Ĭ�Ϲ��캯��(" << *this << ")\n";
}


String::String(const String& other)
{
	init(other.buffer);
	cout << "�������캯����" << *this << ")\n";
}

String::String(String&& other)                  //��&&����ʾ����ֵ���á�����ֵ���ÿ��԰󶨵���ֵ�������ܰ󶨵���ֵ����
{
	// ��other�����Ϳ��������this
	buffer = nullptr;
	buffer = other.buffer;
	_length = other._length;
	other.buffer = nullptr;
	other._length = 0;
	cout << "�ƶ����캯����" << *this << ")\n";
}

String::~String()
{
	delete[] buffer;
	cout << "����������" << *this << ")\n";
}

/*
* �������캯��ʹ�ô�������ֵ����һ���µĶ����ʵ��
* ��ֵ������ǽ������ֵ���Ƹ�һ���Ѿ����ڵ�ʵ��
*/
String& String::operator=(const String& other)
{
	if (this != &other){
		delete[] buffer;
		init(other.buffer);
	}

	cout << "������ֵ������" << *this << ")\n";
	return *this;
}

/*
* �ƶ���ֵ�������Ѳ����������Ķ��������Ȩת�Ƶ�thisָ��Ķ���
* �Ϳ�other���������
*/
String& String::operator=(String&& other) 
{
	if (this != &other){
		buffer = nullptr;
		buffer = other.buffer;
		_length = other._length;
		other.buffer = nullptr;
		other._length = 0;
	}
	cout << "�ƶ���ֵ������" << *this << ")\n";
	return *this;
}

char& String::operator[](size_t index)
{
	if (index >= _length)
		throw std::out_of_range("index out of range");
	else
	{
		return buffer[index];
	}
}

bool String::operator==(const String& other) {
	if (_length != other._length) {
		return false;
	}
	else {
		return 0 == std::strcmp(buffer, other.buffer);
	}
}

/*
* �����Ƿ��ض������Ƿ��ض�������
* ������������ں����д�������ʱ������Ҫʹ������
* ����������ص���ͨ�����û�ָ�봫�ݸ����Ķ�����Ӧ�������÷��ض���
* ����ȴ���һ������Ȼ�󷵻ظö���ĸ����������ʹ�÷��ض���
*/
String String::operator+(const String& other)
{
	String _str;
	if (nullptr == buffer)
		_str = other;
	else if (nullptr == other.buffer)
		_str = *this;
	else{
		_str.buffer = new char[_length + other._length + 1];
		std::strcpy(_str.buffer, buffer);
		std::strcat(_str.buffer, other.buffer);
		_str._length = std::strlen(_str.buffer);
	}
	return _str;
}

ostream& operator<<(ostream &output, const String& str)
{
	if (nullptr == str.buffer) output << "";
	else
		output << str.buffer;

	return output;
}

istream& operator>>(istream &input, String& str) {
	input >> str.buffer;
	return input;
}

