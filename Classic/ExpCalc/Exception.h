#pragma once

#include <string>
#include <exception>

class Exception : public std::exception		// 继承自标准异常类
{
public:
	//为了防止转换构造函数,声明为显示构造函数
	explicit Exception(const char* message);
	explicit Exception(const std::string& message);
	// throw()表示不抛出异常
	virtual ~Exception() throw();					// 虚析构函数是为了能够调用派生类的析构函数防止内存泄漏
	virtual char const* what() const throw();		// 覆盖基类的what方法
	const char* StackTrace() const throw();			// 栈信息回溯
private:
	void FillStackTrace();		// 将栈信息写入
	std::string m_Message;
	std::string m_StackTrace;
}; 

// 语法错误类
class SyntaxError : public Exception
{
public:
	explicit SyntaxError(const char* message);
	explicit SyntaxError(const std::string& message);
	virtual ~SyntaxError() throw();
private:
	std::string m_Message;
};


//文件流出错
class FileStreamError :public Exception
{
public:
	explicit FileStreamError(const char* message);
	explicit FileStreamError(const std::string& message);
	virtual ~FileStreamError() throw();
private:
	std::string m_Message;
};
