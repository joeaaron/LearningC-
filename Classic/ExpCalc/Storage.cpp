#include "Storage.h"
#include "SymbolTable.h"
#include "Exception.h"

Storage::Storage(SymbolTable& symtable)
{
	AddConstants(symtable);			// ��ӳ���
}

// ��ձ�������
void Storage::Clear()
{
	m_Cells.clear();
	m_Inits.clear();
}

// ���ӳ���
void Storage::AddConstants(SymbolTable& symtable)
{
	// ���ӳ���e
	unsigned int id = symtable.Add("e");
	AddValue(id, exp(1.0));

	// ���ӳ���pi
	id = symtable.Add("pi");
	AddValue(id, 2.0*acos(0.0));
}

// �жϱ����Ƿ��ʼ��
bool Storage::IsInit(unsigned int id) const
{
	return id < m_Cells.size() && m_Inits[id];
}

// ��ȡ�������߳�����ֵ
double Storage::GetValue(unsigned int id) const
{
	if (id >= m_Cells.size())
		throw Exception("Internal Error: this variable is not defined.");
	return m_Cells[id];
}

// ���ó����������ֵ
void Storage::SetValue(unsigned int id, double value)
{
	if (id < 0)
		throw Exception("Internal Error: this variable is not defined.");
	// ������Ѿ����ڵı�����ֱ�Ӹ�ֵ
	if (id < m_Cells.size())
	{
		m_Cells[id] = value;
		m_Inits[id] = true;
	}
	else
		AddValue(id, value);
}

// ��ӳ����������ֵ
void Storage::AddValue(unsigned int id, double value)
{
	// ���һ�������Ͷ����һ���ռ�
	m_Cells.resize(id + 1);
	m_Inits.resize(id + 1);
	m_Cells[id] = value;
	m_Inits[id] = true;
}

// ���л������������
void Storage::Serialize(Serializer& out) const
{
	out << m_Cells.size();
	for (unsigned int i = 0; i < m_Cells.size(); i++)
	{
		out << m_Cells[i] << m_Inits[i];
	}
}

//�����л�������������
void Storage::DeSerialize(DeSerializer& in)
{
	m_Cells.clear();
	m_Inits.clear();
	unsigned int uSize;
	in >> uSize;
	m_Cells.resize(uSize);
	m_Inits.resize(uSize);
	for (unsigned int i = 0; i < uSize; i++)
	{
		double value;
		bool b;
		in >> value >> b;
		m_Cells[i] = value;
		m_Inits[i] = b;
	}
}