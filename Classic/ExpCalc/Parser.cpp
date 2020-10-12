#include <iostream>
#include <cassert>
#include <sstream>
#include "Parser.h"
#include "Scanner.h"
#include "Node.h"
#include "Calc.h"
#include "Exception.h"
#include "DebugNew.h"

Parser::Parser(Scanner& scanner, Calc& calc)
	:m_Scanner(scanner), m_Calc(calc), m_Tree(0), m_Status(STATUS_OK)
{

}
Parser::~Parser()
{

}

//解析表达式
EStatus Parser::Parse()
{
	m_Tree = Expression();
	if (!m_Scanner.IsDone()) //判断是否解析完整个表达式
	{
		m_Status = STATUS_ERROR;
	}
	return m_Status;
}


//计算表达式的值
double Parser::Calculate() const
{
	assert(m_Tree.get() != 0);			//因为智能指针不能等于0,所以不能跟0比较,用它的原生指针才可以
	return m_Tree->Calc();				//根结点的值就是整个表达式的值
}

//表达式
std::auto_ptr<Node> Parser::Expression()
{
	//一个表达式可以是项+(-)表达式 或者就是一个项 
	//当term返回的时候,把资源所有权转移给了node管理,只要node被释放,则不会出现内存泄漏
	std::auto_ptr<Node> node = Term(); //这是一个左结点
	EToken token = m_Scanner.Token();//当前扫描到的状态
	if (token == TOKEN_PLUS || token == TOKEN_MINUS)
	{
		std::auto_ptr<MultipleNode>	multipleNode(new SumNode(node));
		do
		{
			m_Scanner.ScanChar();//扫描下一个字符
			std::auto_ptr<Node> nextNode = Term();
			multipleNode->AppendChild(nextNode, (token == TOKEN_PLUS));
			token = m_Scanner.Token();//更新当前字符状态
		} while (token == TOKEN_PLUS || token == TOKEN_MINUS);
		node = multipleNode;
	}
	else if (token == TOKEN_ASSIGN)
	{
		m_Scanner.ScanChar();
		std::auto_ptr<Node> nodeRight = Expression();
		if (node->IsLvalue()) //如果左结点是左值的话就赋值
		{
			node = std::auto_ptr<Node>(new AssignNode(node, nodeRight));
		}
		else
		{
			m_Status = STATUS_ERROR;
			throw SyntaxError("Syntax Error: The left - hand side of an assignment must be a variable ");
		}
	}
	//如果仅仅只是一个项,则直接返回node
	return node;
}

//项
std::auto_ptr<Node> Parser::Term()
{
	//一个项可以是 因式 *(/) 项 或者就是一个因式
	std::auto_ptr<Node> node = Factor();
	EToken token = m_Scanner.Token();			//当前扫描到的状态

	if (token == TOKEN_MULTIPLY || token == TOKEN_DIVIDE)
	{
		std::auto_ptr<MultipleNode>	multipleNode(new ProductNode(node));
		do
		{
			m_Scanner.ScanChar();				//扫描下一个字符
			std::auto_ptr<Node> nextNode = Factor();
			multipleNode->AppendChild(nextNode, (token == TOKEN_MULTIPLY));
			token = m_Scanner.Token();			//更新当前字符状态
		} while (token == TOKEN_MULTIPLY || token == TOKEN_DIVIDE);
		node = multipleNode;
	}
	//如果仅仅只是一个因式,则直接返回node
	return node;
}
//因式
std::auto_ptr<Node> Parser::Factor()
{
	//一个因式可以是 数字 标识符 负号 (表达式)
	std::auto_ptr<Node> node;
	EToken token = m_Scanner.Token();//当前扫描到的状态

	if (token == TOKEN_LPARENTHESIS)
	{
		m_Scanner.ScanChar();//扫描到 (
		node = Expression();//递归解析
		if (m_Scanner.Token() == TOKEN_RPARENTHESIS)
		{
			m_Scanner.ScanChar();// 扫描到)
		}
		else
		{
			m_Status = STATUS_ERROR;
			throw SyntaxError("Syntax Error: miss ')'!");
		}
	}
	else if (token == TOKEN_IDENTIFIER)
	{
		std::string symbol = m_Scanner.GetSymbol();	 //从符号表(变量表)中获取变量
		unsigned int id = m_Calc.FindSymbol(symbol); //找到该变量的ID
		m_Scanner.ScanChar();

		if (m_Scanner.Token() == TOKEN_LPARENTHESIS) //如果标识符后面跟括号则是函数调用(A())
		{
			m_Scanner.ScanChar();
			node = Expression();		 //函数里面可能也是一个表达式(A(2+3))
			if (m_Scanner.Token() == TOKEN_RPARENTHESIS)
			{
				m_Scanner.ScanChar();
				if (id != SymbolTable::IDNOTFOUND && m_Calc.IsFunction(id))
				{
					node = std::auto_ptr<Node>(new FunctionNode(node, m_Calc.GetFunction(id)));
				}
				else
				{
					m_Status = STATUS_ERROR;
					std::ostringstream oss;	 //输出字符串流
					oss << "Syntax Error: Unknow function \"" << symbol << "\"";
					throw SyntaxError(oss.str());
				}
			}
			else
			{
				m_Status = STATUS_ERROR;
				throw SyntaxError("Syntax Error: miss ')' in a function call.");
			}

		}
		else
		{
			if (id == SymbolTable::IDNOTFOUND)			 //如果变量还没有存放到storage则存放进去
			{
				id = m_Calc.AddSymbol(symbol);  //返回新加入变量的ID
			}
			node = std::auto_ptr<Node>(new VariableNode(id, m_Calc.GetStorage()));
		}

	}
	else if (token == TOKEN_NUMBER)
	{
		node = std::auto_ptr<Node>(new NumberNode(m_Scanner.Number()));
		m_Scanner.ScanChar();
	}
	else if (token == TOKEN_MINUS)
	{
		m_Scanner.ScanChar();
		node = std::auto_ptr<Node>(new UminusNode(Factor()));
	}
	else
	{
		m_Status = STATUS_ERROR;
		throw SyntaxError("Syntax Error: not a valid expression.");
	}
	return node;
}