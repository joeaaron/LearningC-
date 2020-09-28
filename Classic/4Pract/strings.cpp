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
	cout << "默认构造函数(" << *this << ")\n";
}


String::String(const String& other)
{
	init(other.buffer);
	cout << "拷贝构造函数（" << *this << ")\n";
}

String::String(String&& other)                  //”&&”表示“右值引用”。右值引用可以绑定到右值（但不能绑定到左值）：
{
	// 把other对象掏空用来填充this
	buffer = nullptr;
	buffer = other.buffer;
	_length = other._length;
	other.buffer = nullptr;
	other._length = 0;
	cout << "移动构造函数（" << *this << ")\n";
}

String::~String()
{
	delete[] buffer;
	cout << "析构函数（" << *this << ")\n";
}

/*
* 拷贝构造函数使用传入对象的值生成一个新的对象的实例
* 赋值运算符是将对象的值复制给一个已经存在的实例
*/
String& String::operator=(const String& other)
{
	if (this != &other){
		delete[] buffer;
		init(other.buffer);
	}

	cout << "拷贝赋值操作（" << *this << ")\n";
	return *this;
}

/*
* 移动赋值操作即把参数传进来的对象的所有权转移到this指向的对象
* 掏空other对象的所有
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
	cout << "移动赋值操作（" << *this << ")\n";
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
* 关于是返回对象本身还是返回对象引用
* 如果函数返回在函数中创建的临时对象，则不要使用引用
* 如果函数返回的是通过引用或指针传递给它的对象，则应当按引用返回对象
* 如果先创建一个对象，然后返回该对象的副本，则可以使用返回对象
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

