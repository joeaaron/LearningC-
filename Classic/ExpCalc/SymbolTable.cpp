#include <algorithm>
#include "SymbolTable.h"
#include "Exception.h"

// ��ӷ���
unsigned int SymbolTable::Add(const std::string& strSymbol)
{
	m_Symbol[strSymbol] = m_uId;     // ���ַ��������Ҷ�ӦID
	return m_uId++;
}

// ���ҷ���
unsigned int SymbolTable::Find(const std::string& strSymbol) const
{
	std::map<const std::string, unsigned int>::const_iterator const_iter = m_Symbol.find(strSymbol);
	return (const_iter != m_Symbol.end()) ? const_iter->second : IDNOTFOUND;
}

// ��ձ���
void SymbolTable::Clear()
{
	m_Symbol.clear();
	m_uId = 0;
}

// ��������
class IsEqualId
{
public:
	IsEqualId(unsigned int id) : m_Id(id){}
	bool operator()(const std::pair<const std::string, unsigned int>& it) const
	{
		return it.second == m_Id;
	}
private:
	unsigned int m_Id;
};

// ���ҷ���
std::string SymbolTable::GetSymbolName(unsigned int id)	const
{
	std::map<const std::string, unsigned int>::const_iterator const_iter;
	const_iter = find_if(m_Symbol.begin(), m_Symbol.end(), IsEqualId(id));
	if (const_iter == m_Symbol.end())
		throw Exception("Internal error: missing entry in symboltable."); //�׳��쳣
	return const_iter->first;
}

// ���ط��ű��еĺ����ͱ�������
unsigned int SymbolTable::GetSymtblSize() const
{
	return m_uId;
}

// ���л������������
void SymbolTable::Serialize(Serializer& out) const
{
	out << m_Symbol.size();			//�Ȱѱ�������д��
	auto it = m_Symbol.begin();
	for (; it != m_Symbol.end(); ++it)
		out << it->first << it->second;
	out << m_uId;
}

// �����л�������������
void SymbolTable::DeSerialize(DeSerializer& in)
{
	m_Symbol.clear();				// ������������ٴ��ļ�����
	unsigned int uSize;
	in >> uSize;
	for (unsigned int i = 0; i < uSize; i++)
	{
		std::string str;
		unsigned int id;
		in >> str >> id;
		m_Symbol[str] = id;
	}
	in >> m_uId;
}