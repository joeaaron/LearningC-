#include <iostream>
#include "Scanner.h"

Scanner::Scanner(std::istream& in)
	:m_In(in)
{
	ScanChar();								// 当创建对象的时候就先扫描一个字符
	m_bIsEmpty = (m_Token == TOKEN_END);	// 第一次扫描遇到回车就是空的
}

// 一个个读取字符
void Scanner::ReadChar()  
{
	m_Look = m_In.get();					// 从标准输入中获取字符
	while (m_Look == ' ' || m_Look == '\t')	// 忽略空格
		m_Look = m_In.get();
}

// 判断是否是命令
bool Scanner::IsCommand() const
{
	return m_Token == TOKEN_COMMAND;
}

// 扫描一个一个字符
void Scanner::ScanChar()
{
	// 解析字符
	ReadChar();
	switch (m_Look)
	{
	case '#':
		m_Token = TOKEN_COMMAND;
		break;
	case '+':
		m_Token = TOKEN_PLUS;
		break;
	case '-':
		m_Token = TOKEN_MINUS;
		break;
	case '*':
		m_Token = TOKEN_MULTIPLY;
		break;
	case '/':
		m_Token = TOKEN_DIVIDE;
		break;
	case '=':
		m_Token = TOKEN_ASSIGN;
		break;
	case '(':
		m_Token = TOKEN_LPARENTHESIS;
		break;
	case ')':
		m_Token = TOKEN_RPARENTHESIS;
		break;
	case '0':case'1': case '2':case'3':case '4':
	case'5':case '6':case'7':case '8':case'9':
	case '.':
		m_Token = TOKEN_NUMBER;
		m_In.putback(m_Look);		// 把读取的放回去
		m_In >> m_Number;			// 把输入流的数字放到number中
		break;
	case '\0':case'\n':case '\r':case EOF:
		m_Token = TOKEN_END;//如果是以上字符则表示解析结束
		break;
	default:
		// 变量可以是字母或者下划线开头
		if (isalpha(m_Look) || m_Look == '_')
		{
			m_Token = TOKEN_IDENTIFIER;
			m_SymBol.erase();		//先把临时变量清除,再获取新变量
			// 获取变量名
			do
			{
				m_SymBol += m_Look;
				m_Look = m_In.get();
			} while (isalnum(m_Look) || m_Look == '_');
			m_In.putback(m_Look);//把最后多获取的字符放回流中
		}
		else
			m_Token = TOKEN_ERROR;
		break;
	}
}
 
// 返回数字
double Scanner::Number() const
{
	return m_Number;
}

// 返回扫描状态
EToken Scanner::Token() const
{
	return m_Token;
}

// 获取标识符
std::string Scanner::GetSymbol() const
{
	return m_SymBol;
}

// 判断表达式是否为空
bool Scanner::IsEmpty() const
{
	return m_bIsEmpty;
}

// 判断表达式是否被扫描完
bool Scanner::IsDone() const
{
	return m_Token == TOKEN_END;//如果没有扫描到结束符,则表达式没有被扫描完
}

// 获取命令参数
void Scanner::GetCmdParam()
{
	ReadChar();
	m_SymBol.erase();
	while (!isspace(m_Look))
	{
		m_SymBol += m_Look;		//把命令加入到符号表
		m_Look = m_In.get();
	}
}