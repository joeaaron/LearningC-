#include <iostream>
#include "Calc.h"
//���ұ���
unsigned int Calc::FindSymbol(const std::string& strSymbol) const
{
	return m_Symtable.Find(strSymbol);
}
//���ӱ���
unsigned int Calc::AddSymbol(const std::string& strSymbol)
{
	return m_Symtable.Add(strSymbol);
}

Storage& Calc::GetStorage()
{
	return m_Storage;
}
//�ж��Ƿ��Ǻ���
bool Calc::IsFunction(unsigned int id) const
{
	return id < m_Funtbl.Size();
}
//��ȡ����
PtrFun Calc::GetFunction(unsigned int id) const
{
	return m_Funtbl.GetFunction(id);
}

//�г�����
void Calc::ListFunction() const
{
	for (unsigned int i = 0; i < m_Funtbl.GetSize(); i++)
		std::cout << m_Symtable.GetSymbolName(i) << std::endl;
}


//�г�����
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

//�������
void Calc::Clear()
{
	m_Storage.Clear();
	m_Symtable.Clear();
}