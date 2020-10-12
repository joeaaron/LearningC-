#pragma once

#include <string>
#include <fstream>
// 序列化器
class Serializer
{
public:
	Serializer(const std::string& filename);

	// 写文件的方法
	Serializer& Put(int x);
	Serializer& Put(unsigned int x);
	Serializer& Put(long x);
	Serializer& Put(unsigned long x);
	Serializer& Put(double x);
	Serializer& Put(const std::string& x);
	Serializer& Put(bool x);

	Serializer& operator<<(int x);
	Serializer& operator<<(unsigned int x);
	Serializer& operator<<(long x);
	Serializer& operator<<(unsigned long x);
	Serializer& operator<<(double x);
	Serializer& operator<<(const std::string& x);
	Serializer& operator<<(bool x);

private:
	std::ofstream m_Ostream;     // 文件输出流（写文件）
};

//反序列化器
class DeSerializer
{
public:
	DeSerializer(const std::string& filename);

	//读文件的方法
	DeSerializer& Get(int& x);
	DeSerializer& Get(unsigned int& x);
	DeSerializer& Get(long& x);
	DeSerializer& Get(unsigned long& x);
	DeSerializer& Get(double& x);
	DeSerializer& Get(std::string& x);
	DeSerializer& Get(bool& x);

	DeSerializer& operator>>(int& x);
	DeSerializer& operator>>(unsigned int& x);
	DeSerializer& operator>>(long& x);
	DeSerializer& operator>>(unsigned long& x);
	DeSerializer& operator>>(double& x);
	DeSerializer& operator>>(std::string& x);
	DeSerializer& operator>>(bool& x);

private:
	std::ifstream m_Istream;	// 文件输入流(读文件)
};

// 序列化类
class Serializable
{
public:
	virtual void Serialize(Serializable& out) const = 0;
	virtual void DeSerialize(DeSerializer& in) = 0;
};