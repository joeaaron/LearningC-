#include <cmath>
#include <assert.h>
#include <iostream>
#include "Node.h"
#include "Storage.h"
#include "Exception.h"

BinaryNode::~BinaryNode()
{
	//�ͷŽ��
	//������ָ���Զ��ͷ�
}

UnaryNode::~UnaryNode()
{

}

MultipleNode::MultipleNode(std::auto_ptr<Node>& node)
{
	//��ӵĵ�һ�������������,��ΪUminusNode�����ᴦ����
	AppendChild(node, true);
}

MultipleNode::~MultipleNode()
{

}
//ֱ�ӷ��ظý�����
double NumberNode::Calc() const
{
	return m_Number;
}


//ȡ������
double UminusNode::Calc() const
{
	return -m_Child->Calc();
}

//��������
double FunctionNode::Calc() const
{
	return (*m_pFun)(m_Child->Calc());
}

//�����ӽ��
void MultipleNode::AppendChild(std::auto_ptr<Node>& node, bool positive)
{
	m_Childs.push_back(node);//��Ϊptr_vector������һ�� push_back,֧������ָ�봫��ȥ
	m_Positive.push_back(positive);
}

//������ĺͲ�ֵ
double SumNode::Calc() const
{
	double result = 0.0;			   //MultipleNode���ӽ��
	std::vector<Node*>::const_iterator Chiditer = m_Childs.begin();
	std::vector<bool>::const_iterator Positiveiter = m_Positive.begin();

	for (; Chiditer != m_Childs.end(); Chiditer++, Positiveiter++)
	{
		assert(Positiveiter != m_Positive.end());
		double val = (*Chiditer)->Calc();		  //��MultipleNode�еĽ���ó�����ӻ����
		if (*Positiveiter)
			result += val;
		else
			result -= val;
	}
	return result;
}

//������Ļ���ֵ
double ProductNode::Calc() const
{
	double result = 1.0;		  //��SumNodeһ���ĵ���
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

//�������ļ���
double VariableNode::Calc() const
{
	double x = 0.0;
	//����ñ����Ѿ���ʼ��,���ȡ���������ֵ,�������
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
//�ж��Ƿ�����ֵ
bool VariableNode::IsLvalue() const
{
	return true;//����������ֵ
}
//Ϊ������ֵ
void VariableNode::Assign(double value)
{
	m_Storage.SetValue(m_Id, value);
}

//���������ֵ��Ľ��
double AssignNode::Calc() const
{
	double x = 0.0;
	x = m_Right->Calc();//��ȡ���ҽ���ֵ
	m_Left->Assign(x);//Ȼ��ֵ������ı���
	return x;
}