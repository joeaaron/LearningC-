#pragma once

#include <string>
#include <exception>

class Exception : public std::exception		// �̳��Ա�׼�쳣��
{
public:
	//Ϊ�˷�ֹת�����캯��,����Ϊ��ʾ���캯��
	explicit Exception(const char* message);
	explicit Exception(const std::string& message);
	// throw()��ʾ���׳��쳣
	virtual ~Exception() throw();					// ������������Ϊ���ܹ����������������������ֹ�ڴ�й©
	virtual char const* what() const throw();		// ���ǻ����what����
	const char* StackTrace() const throw();			// ջ��Ϣ����
private:
	void FillStackTrace();		// ��ջ��Ϣд��
	std::string m_Message;
	std::string m_StackTrace;
}; 

// �﷨������
class SyntaxError : public Exception
{
public:
	explicit SyntaxError(const char* message);
	explicit SyntaxError(const std::string& message);
	virtual ~SyntaxError() throw();
private:
	std::string m_Message;
};


//�ļ�������
class FileStreamError :public Exception
{
public:
	explicit FileStreamError(const char* message);
	explicit FileStreamError(const std::string& message);
	virtual ~FileStreamError() throw();
private:
	std::string m_Message;
};
