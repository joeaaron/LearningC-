#pragma once

#include <iostream>

class String
{
private:
	char* buffer;
	size_t _length;
	void init(const char* str);
public:
	String(const char* str = nullptr);   //Ĭ�Ϲ��캯��
	String(const String& other);		 //�������캯��
	String(String&& other);				 //�ƶ����캯��
	~String();							 //��������

	size_t length();
	const char* data();

	char& operator[](size_t index);
	String& operator=(const String& other);
	String& operator=(String&& other);
	String operator+(const String& other);
	bool operator==(const String& other);
	friend std::ostream& operator<<(std::ostream& output, const String& str);
	friend std::istream& operator>>(std::istream& input, String& str);
};