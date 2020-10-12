#pragma once
#include <cassert>
#include <memory>
#include <vector>
#include "FunctionTable.h"
#include "ptr_vector.h"
class Noncopyable
{
protected:
	Noncopyable() {}
	~Noncopyable() {}
private:
	Noncopyable(const Noncopyable&);
	const Noncopyable& operator=(const Noncopyable);
};

// 这是一个抽象类，向派生类提供接口
// 把Node类变为对象语义
class Node : private Noncopyable
{
public:
	virtual double Calc() const = 0;
	virtual bool IsLvalue() const			// 判断是否是左值
	{
		return false;
	}
	virtual void Assign(double)//赋值
	{
		assert(!"Assign called incorrectlly");
	}
	//要把析构函数声明为虚的,
	//防止释放基类指针的时候,不会调用派生类析构函数,造成内存泄漏
	virtual ~Node() {}
};

// 多节点类
class MultipleNode : public Node
{
public:
	MultipleNode(std::auto_ptr<Node>& node);
	~MultipleNode();
	//把+-结点和*/结点添加进去,这样就可以进行正常的自左向右计算
	void AppendChild(std::auto_ptr<Node>& node, bool positive);

protected:
	ptr_vector<Node> m_Childs;			// 利用智能向量管理指针的释放
	std::vector<bool> m_Positive;		// 正负
};

// 求+ - 结点类的和
class SumNode :public MultipleNode
{
public:
	SumNode(std::auto_ptr<Node>& node) :MultipleNode(node) {}
	double Calc() const;
};

// 求* / 结点类的积
class ProductNode :public MultipleNode
{
public:
	ProductNode(std::auto_ptr<Node>& node) :MultipleNode(node) {}
	double Calc() const;
};

// 数字类结点
class NumberNode : public Node
{
public:
	NumberNode(double number) : m_Number(number) {}
	double Calc() const;		// 覆盖基类（抽象类）的方法
protected:
	const double m_Number;
};

// 二元运算结点
class BinaryNode :public Node
{
public:							// 引用传递可以减少一次智能指针的拷贝构造
	BinaryNode(std::auto_ptr<Node>& left, std::auto_ptr<Node>& right) :m_Left(left), m_Right(right) {}
	~BinaryNode();
protected:
	//Node* const m_Left;		//指针不能改变,指针所指向的内容能改变
	//Node* const m_Right;
	std::auto_ptr<Node> m_Left; //利用智能指针来防止内存泄漏,悬空指针,重复释放
	std::auto_ptr<Node> m_Right;
};

// 一元运算结点
class UnaryNode : public Node
{
public:
	UnaryNode(std::auto_ptr<Node>& child) :m_Child(child) {}
	~UnaryNode();
protected:
	std::auto_ptr<Node> m_Child; //一元运算符(-)只有一个子结点

};

// 函数结点
class FunctionNode : public UnaryNode
{
public:
	FunctionNode(std::auto_ptr<Node>& child, PtrFun pfun)
		:UnaryNode(child), m_pFun(pfun)  {}

	double Calc() const;
private:
	PtrFun m_pFun;
};


//取负运算结点
class UminusNode :public UnaryNode
{
public:
	UminusNode(std::auto_ptr<Node>& child) :UnaryNode(child) {}
public:
	double Calc() const;
};

class Storage;

// 变量结点
class VariableNode :public Node
{
public:
	VariableNode(unsigned int id, Storage& storage)
		: m_Id(id), m_Storage(storage) {}

	double Calc() const;
	bool IsLvalue() const;				 //判断是否是左值结点
	void Assign(double value);			 //变量赋值操作
private:
	const unsigned int m_Id;
	Storage& m_Storage;

};

//赋值结点
class AssignNode :public BinaryNode
{
public:
	AssignNode(std::auto_ptr<Node>& left, std::auto_ptr<Node>& right)
		:BinaryNode(left, right)
	{
		assert(m_Left->IsLvalue());		//非变量不能赋值
	}
	double Calc() const;
};