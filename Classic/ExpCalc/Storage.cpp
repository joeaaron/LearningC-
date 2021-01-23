#include "Storage.h"
#include "SymbolTable.h"
#include "Exception.h"

Storage::Storage(SymbolTable& symtable)
{
	AddConstants(symtable);			// 添加常量
}

// 清空变量常量
void Storage::Clear()
{
	m_Cells.clear();
	m_Inits.clear();
}

// 增加常量
void Storage::AddConstants(SymbolTable& symtable)
{
	// 增加常量e
	unsigned int id = symtable.Add("e");
	AddValue(id, exp(1.0));

	// 增加常量pi
	id = symtable.Add("pi");
	AddValue(id, 2.0*acos(0.0));
}

// 判断变量是否初始化
bool Storage::IsInit(unsigned int id) const
{
	return id < m_Cells.size() && m_Inits[id];
}

// 获取变量或者常量的值
double Storage::GetValue(unsigned int id) const
{
	if (id >= m_Cells.size())
		throw Exception("Internal Error: this variable is not defined.");
	return m_Cells[id];
}

// 设置常量或变量的值
void Storage::SetValue(unsigned int id, double value)
{
	if (id < 0)
		throw Exception("Internal Error: this variable is not defined.");
	// 如果是已经存在的变量则直接赋值
	if (id < m_Cells.size())
	{
		m_Cells[id] = value;
		m_Inits[id] = true;
	}
	else
		AddValue(id, value);
}

// 添加常量或变量的值
void Storage::AddValue(unsigned int id, double value)
{
	// 添加一个变量就多分配一个空间
	m_Cells.resize(id + 1);
	m_Inits.resize(id + 1);
	m_Cells[id] = value;
	m_Inits[id] = true;
}

// 序列化方法保存变量
void Storage::Serialize(Serializer& out) const
{
	out << m_Cells.size();
	for (unsigned int i = 0; i < m_Cells.size(); i++)
	{
		out << m_Cells[i] << m_Inits[i];
	}
}

//反序列化方法保存数据
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