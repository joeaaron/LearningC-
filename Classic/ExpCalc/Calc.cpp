#include <iostream>
#include "Calc.h"
//查找变量
unsigned int Calc::FindSymbol(const std::string& strSymbol) const
{
	return m_Symtable.Find(strSymbol);
}
//增加变量
unsigned int Calc::AddSymbol(const std::string& strSymbol)
{
	return m_Symtable.Add(strSymbol);
}

Storage& Calc::GetStorage()
{
	return m_Storage;
}
//判断是否是函数
bool Calc::IsFunction(unsigned int id) const
{
	return id < m_Funtbl.Size();
}
//获取函数
PtrFun Calc::GetFunction(unsigned int id) const
{
	return m_Funtbl.GetFunction(id);
}

//列出函数
void Calc::ListFunction() const
{
	for (unsigned int i = 0; i < m_Funtbl.GetSize(); i++)
		std::cout << m_Symtable.GetSymbolName(i) << std::endl;
}


//列出变量
void Calc::ListVarible() const
{
	for (unsigned int i = m_Funtbl.GetSize(); i < m_Symtable.GetSymtblSize(); i++)
	{
		std::cout << m_Symtable.GetSymbolName(i) << " = ";
		if (m_Storage.IsInit(i))
			std::cout << m_Storage.GetValue(i) << std::endl;
		else
			std::cout << "?" << std::endl;
	}

}

void Calc::Serialize(Serializer& out)
{
	m_Symtable.Serialize(out);
	m_Storage.Serialize(out);
}
void Calc::DeSerialize(DeSerializer& in)
{
	m_Symtable.DeSerialize(in);
	m_Storage.DeSerialize(in);
}

//清除变量
void Calc::Clear()
{
	m_Storage.Clear();
	m_Symtable.Clear();
}