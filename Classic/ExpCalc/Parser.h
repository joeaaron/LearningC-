#pragma once

#include <memory>

// 解析器的状态
enum EStatus
{
	STATUS_OK,
	STATUS_QUIT,
	STATUS_ERROR
};

// 前置声明
class Calc;
class Node;
class Scanner;

// 解析表达式类
class Parser
{
public:
	Parser(Scanner& scanner, Calc& calc);
	~Parser();
	EStatus Parse();					// 解析表达式并且生成表达式树
	double Calculate() const;			// 计算结果
	std::auto_ptr<Node> Expression();	// 结点为表达式
	std::auto_ptr<Node> Term();			// 结点为项
	std::auto_ptr<Node> Factor();		// 结点为因子
private:
	Scanner& m_Scanner;
	Calc& m_Calc;
	std::auto_ptr<Node> m_Tree;			// 表达式树
	EStatus m_Status;

};