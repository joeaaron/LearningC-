#pragma once

// 标识扫描到的是什么字符
enum EToken
{
	TOKEN_END,					// 终止字符
	TOKEN_ERROR,				// 扫描到错误字符
	TOKEN_NUMBER,				// 数字 
	TOKEN_PLUS,					// +
	TOKEN_MINUS,				// -
	TOKEN_MULTIPLY,				// *
	TOKEN_DIVIDE,				// /
	TOKEN_LPARENTHESIS,			// (
	TOKEN_RPARENTHESIS,			// )
	TOKEN_IDENTIFIER,			// 标识符
	TOKEN_ASSIGN,				// 赋值号
	TOKEN_COMMAND				// 命令
};

// 扫描表达式类
class Scanner
{
public:
	explicit Scanner(std::istream& in);	     // 防止隐式转换
	void ScanChar();						 // 扫描一个字符
	double Number() const;					 // 返回数字
	EToken Token() const;					 // 返回扫描到的字符类型
	std::string GetSymbol() const;			 // 获取标识符
	bool IsEmpty() const;					 // 判断表达式是否为空
	bool IsDone() const;					 // 判断是否扫描完整个表达式
	bool IsCommand() const;					 // 判断是否是命令
	void GetCmdParam();						 // 获取命令参数

private:
	void ReadChar();						 // 从输入流中一个个字符解析
	std::istream& m_In;						 // 输入流
	double m_Number;						 // 数字
	EToken m_Token;							 // 标识扫描到的是什么字符
	std::string m_SymBol;					 // 变量标识符
	bool m_bIsEmpty;						 // 判断表达式是否为空
	int m_Look;								 // 当前解析到的字符
};