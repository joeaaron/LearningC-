#include "Serial.h"
#include "Exception.h"

using std::ios;
const int True = 0x0000ffff;
const int False = 0x0000fffe;

Serializer::Serializer(const std::string& filename)
	:m_Ostream(filename.c_str(), ios::binary)		// 打开文件
{
	if (!m_Ostream)
		throw FileStreamError("open file failed.");
}

Serializer& Serializer::Put(int x)
{
	m_Ostream.write(reinterpret_cast<char*>(&x), sizeof(int));
	if (m_Ostream.bad())
		throw FileStreamError("write file failed.");
	return *this;
}

Serializer& Serializer::Put(unsigned int x)
{
	m_Ostream.write(reinterpret_cast<char*>(&x), sizeof(unsigned int));
	if (m_Ostream.bad())
		throw FileStreamError("write file failed.");
	return *this;
}

Serializer& Serializer::Put(long x)
{
	m_Ostream.write(reinterpret_cast<char*>(&x), sizeof(long));
	if (m_Ostream.bad())
		throw FileStreamError("write file failed.");
	return *this;
}

Serializer& Serializer::Put(unsigned long x)
{
	m_Ostream.write(reinterpret_cast<char*>(&x), sizeof(unsigned long));
	if (m_Ostream.bad())
		throw FileStreamError("write file failed.");
	return *this;
}

Serializer& Serializer::Put(double x)
{
	m_Ostream.write(reinterpret_cast<char*>(&x), sizeof(double));
	if (m_Ostream.bad())
		throw FileStreamError("write file failed.");
	return *this;
}

Serializer& Serializer::Put(const std::string& x)
{
	int nLen = x.length();
	Put(nLen);
	m_Ostream.write(x.data(), nLen);
	if (m_Ostream.bad())
		throw FileStreamError("write file failed.");
	return *this;
}

Serializer& Serializer::Put(bool x)
{
	int n = x ? True : False;          //统一把bool当成整型写入
	Put(n);
	return *this;
}

Serializer& Serializer::operator<<(int x)
{
	return Put(x);
}

Serializer& Serializer::operator<<(unsigned int x)
{
	return Put(x);
}

Serializer& Serializer::operator<<(long x)
{
	return Put(x);
}

Serializer& Serializer::operator<<(unsigned long x)
{
	return Put(x);
}

Serializer& Serializer::operator<<(double x)
{
	return Put(x);
}

Serializer& Serializer::operator<<(const std::string& x)
{
	return Put(x);
}

Serializer& Serializer::operator<<(bool x)
{
	return Put(x);
}

DeSerializer::DeSerializer(const std::string& filename)
	: m_Istream(filename.c_str(), ios::binary)
{
	if (!m_Istream)
		throw FileStreamError("open file failed.");
}

DeSerializer& DeSerializer::Get(int& x)
{
	if (m_Istream.eof())
		throw FileStreamError("Unexpected end of file.");
	m_Istream.read(reinterpret_cast<char*>(&x), sizeof(int));
	if (m_Istream.bad())
		throw FileStreamError("read file failed.");
	return *this;
}

DeSerializer& DeSerializer::Get(unsigned int& x)
{
	if (m_Istream.eof())
		throw FileStreamError("Unexpected end of file.");
	m_Istream.read(reinterpret_cast<char*>(&x), sizeof(unsigned int));
	if (m_Istream.bad())
		throw FileStreamError("read file failed.");
	return *this;
}

DeSerializer& DeSerializer::Get(long& x)
{
	if (m_Istream.eof())
		throw FileStreamError("Unexpected end of file.");
	m_Istream.read(reinterpret_cast<char*>(&x), sizeof(long));
	if (!m_Istream)
		throw FileStreamError("read file failed.");
	return *this;
}

DeSerializer& DeSerializer::Get(unsigned long& x)
{
	if (m_Istream.eof())
		throw FileStreamError("Unexpected end of file.");
	m_Istream.read(reinterpret_cast<char*>(&x), sizeof(unsigned long));
	if (m_Istream.bad())
		throw FileStreamError("read file failed.");
	return *this;
}

DeSerializer& DeSerializer::Get(double& x)
{
	if (m_Istream.eof())
		throw FileStreamError("Unexpected end of file.");
	m_Istream.read(reinterpret_cast<char*>(&x), sizeof(double));
	if (m_Istream.bad())
		throw FileStreamError("read file failed.");
	return *this;
}

DeSerializer& DeSerializer::Get(std::string& x)
{
	int nLen = 0;
	Get(nLen);	   //先读取字符串长度
	if (m_Istream.eof())
		throw FileStreamError("Unexpected end of file.");
	x.resize(nLen);
	m_Istream.read(&x[0], nLen);
	if (m_Istream.bad())
		throw FileStreamError("read file failed.");
	return *this;
}

DeSerializer& DeSerializer::Get(bool& x)
{
	int n;
	Get(n);
	if (n == True)
		x = true;
	else if (n == False)
		x = false;
	else
		throw FileStreamError("data error.");

	return *this;
}

DeSerializer& DeSerializer::operator>>(int& x)
{
	return Get(x);
}

DeSerializer& DeSerializer::operator>>(unsigned int& x)
{
	return Get(x);
}

DeSerializer& DeSerializer::operator>>(long& x)
{
	return Get(x);
}

DeSerializer& DeSerializer::operator>>(unsigned long& x)
{
	return Get(x);

}

DeSerializer& DeSerializer::operator>>(double& x)
{
	return Get(x);

}

DeSerializer& DeSerializer::operator>>(std::string& x)
{
	return Get(x);
}

DeSerializer& DeSerializer::operator>>(bool& x)
{
	return Get(x);
}
