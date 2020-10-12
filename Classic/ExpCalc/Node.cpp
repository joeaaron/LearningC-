#include <cmath>
#include <assert.h>
#include <iostream>
#include "Node.h"
#include "Storage.h"
#include "Exception.h"

BinaryNode::~BinaryNode()
{
	//释放结点
	//由智能指针自动释放
}

UnaryNode::~UnaryNode()
{

}

MultipleNode::MultipleNode(std::auto_ptr<Node>& node)
{
	//添加的第一个结点总是正的,因为UminusNode这个类会处理负号
	AppendChild(node, true);
}

MultipleNode::~MultipleNode()
{

}
//直接返回该结点的数
double NumberNode::Calc() const
{
	return m_Number;
}


//取负运算
double UminusNode::Calc() const
{
	return -m_Child->Calc();
}

//函数运算
double FunctionNode::Calc() const
{
	return (*m_pFun)(m_Child->Calc());
}

//加入子结点
void MultipleNode::AppendChild(std::auto_ptr<Node>& node, bool positive)
{
	m_Childs.push_back(node);//因为ptr_vector重载了一个 push_back,支持智能指针传进去
	m_Positive.push_back(positive);
}

//计算结点的和差值
double SumNode::Calc() const
{
	double result = 0.0;			   //MultipleNode的子结点
	std::vector<Node*>::const_iterator Chiditer = m_Childs.begin();
	std::vector<bool>::const_iterator Positiveiter = m_Positive.begin();

	for (; Chiditer != m_Childs.end(); Chiditer++, Positiveiter++)
	{
		assert(Positiveiter != m_Positive.end());
		double val = (*Chiditer)->Calc();		  //将MultipleNode中的结点拿出来相加或相减
		if (*Positiveiter)
			result += val;
		else
			result -= val;
	}
	return result;
}

//计算结点的积商值
double ProductNode::Calc() const
{
	double result = 1.0;		  //和SumNode一样的道理
	std::vector<Node*>::const_iterator Chiditer = m_Childs.begin();
	std::vector<bool>::const_iterator Positiveiter = m_Positive.begin();

	for (; Chiditer != m_Childs.end(); Chiditer++, Positiveiter++)
	{
		assert(Positiveiter != m_Positive.end());
		double val = (*Chiditer)->Calc();
		if (*Positiveiter)
			result *= val;
		else
		{
			assert(val != 0.0);
			result /= val;
		}
	}
	return result;
}

//变量结点的计算
double VariableNode::Calc() const
{
	double x = 0.0;
	//如果该变量已经初始化,则获取这个变量的值,否则出错
	if (m_Storage.IsInit(m_Id))
	{
		x = m_Storage.GetValue(m_Id);
	}
	else
	{
		throw SyntaxError("Syntax Error: use of uninitialized variable.");
	}
	return x;
}
//判断是否是左值
bool VariableNode::IsLvalue() const
{
	return true;//变量都是左值
}
//为变量赋值
void VariableNode::Assign(double value)
{
	m_Storage.SetValue(m_Id, value);
}

//计算变量赋值后的结果
double AssignNode::Calc() const
{
	double x = 0.0;
	x = m_Right->Calc();//先取出右结点的值
	m_Left->Assign(x);//然后赋值给左结点的变量
	return x;
}