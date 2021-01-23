#include "Exception.h"

Exception::Exception(const char* message) :m_Message(message)
{

}
Exception::Exception(const std::string& message) : m_Message(message)
{

}
Exception::~Exception() throw()
{

}
SyntaxError::SyntaxError(const char* message) :Exception(message)
{

}
SyntaxError::SyntaxError(const std::string& message) : Exception(message)
{

}
SyntaxError::~SyntaxError() throw()
{

}
FileStreamError::FileStreamError(const char* message) :Exception(message)
{

}
FileStreamError::FileStreamError(const std::string& message) : Exception(message)
{

}
FileStreamError::~FileStreamError() throw()
{

}
//���������Ϣ
char const* Exception::what() const	throw()
{
	return m_Message.c_str();
}

//��˷ջ��Ϣ
const char* Exception::StackTrace() const throw()
{
	return m_StackTrace.c_str();
}

void Exception::FillStackTrace()
{
	//������linux����ջ����

	/*const int iLen = 200;
	void* buffer[iLen];
	int nPtrs = ::backtrace(buffer, iLen);
	char** strings = ::backtrace_symbols(buffer, nPtrs);
	if (strings)
	{
	for (int i = 0; i < nPtrs; i++)
	{
	m_StackTrace.append(strings[i]);
	m_StackTrace.push_back('\n');
	}
	free(strings);

	}  */
}