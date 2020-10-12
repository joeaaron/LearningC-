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

// ����һ�������࣬���������ṩ�ӿ�
// ��Node���Ϊ��������
class Node : private Noncopyable
{
public:
	virtual double Calc() const = 0;
	virtual bool IsLvalue() const			// �ж��Ƿ�����ֵ
	{
		return false;
	}
	virtual void Assign(double)//��ֵ
	{
		assert(!"Assign called incorrectlly");
	}
	//Ҫ��������������Ϊ���,
	//��ֹ�ͷŻ���ָ���ʱ��,���������������������,����ڴ�й©
	virtual ~Node() {}
};

// ��ڵ���
class MultipleNode : public Node
{
public:
	MultipleNode(std::auto_ptr<Node>& node);
	~MultipleNode();
	//��+-����*/�����ӽ�ȥ,�����Ϳ��Խ����������������Ҽ���
	void AppendChild(std::auto_ptr<Node>& node, bool positive);

protected:
	ptr_vector<Node> m_Childs;			// ����������������ָ����ͷ�
	std::vector<bool> m_Positive;		// ����
};

// ��+ - �����ĺ�
class SumNode :public MultipleNode
{
public:
	SumNode(std::auto_ptr<Node>& node) :MultipleNode(node) {}
	double Calc() const;
};

// ��* / �����Ļ�
class ProductNode :public MultipleNode
{
public:
	ProductNode(std::auto_ptr<Node>& node) :MultipleNode(node) {}
	double Calc() const;
};

// ��������
class NumberNode : public Node
{
public:
	NumberNode(double number) : m_Number(number) {}
	double Calc() const;		// ���ǻ��ࣨ�����ࣩ�ķ���
protected:
	const double m_Number;
};

// ��Ԫ������
class BinaryNode :public Node
{
public:							// ���ô��ݿ��Լ���һ������ָ��Ŀ�������
	BinaryNode(std::auto_ptr<Node>& left, std::auto_ptr<Node>& right) :m_Left(left), m_Right(right) {}
	~BinaryNode();
protected:
	//Node* const m_Left;		//ָ�벻�ܸı�,ָ����ָ��������ܸı�
	//Node* const m_Right;
	std::auto_ptr<Node> m_Left; //��������ָ������ֹ�ڴ�й©,����ָ��,�ظ��ͷ�
	std::auto_ptr<Node> m_Right;
};

// һԪ������
class UnaryNode : public Node
{
public:
	UnaryNode(std::auto_ptr<Node>& child) :m_Child(child) {}
	~UnaryNode();
protected:
	std::auto_ptr<Node> m_Child; //һԪ�����(-)ֻ��һ���ӽ��

};

// �������
class FunctionNode : public UnaryNode
{
public:
	FunctionNode(std::auto_ptr<Node>& child, PtrFun pfun)
		:UnaryNode(child), m_pFun(pfun)  {}

	double Calc() const;
private:
	PtrFun m_pFun;
};


//ȡ��������
class UminusNode :public UnaryNode
{
public:
	UminusNode(std::auto_ptr<Node>& child) :UnaryNode(child) {}
public:
	double Calc() const;
};

class Storage;

// �������
class VariableNode :public Node
{
public:
	VariableNode(unsigned int id, Storage& storage)
		: m_Id(id), m_Storage(storage) {}

	double Calc() const;
	bool IsLvalue() const;				 //�ж��Ƿ�����ֵ���
	void Assign(double value);			 //������ֵ����
private:
	const unsigned int m_Id;
	Storage& m_Storage;

};

//��ֵ���
class AssignNode :public BinaryNode
{
public:
	AssignNode(std::auto_ptr<Node>& left, std::auto_ptr<Node>& right)
		:BinaryNode(left, right)
	{
		assert(m_Left->IsLvalue());		//�Ǳ������ܸ�ֵ
	}
	double Calc() const;
};