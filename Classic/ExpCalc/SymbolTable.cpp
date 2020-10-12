#include <algorithm>
#include "SymbolTable.h"
#include "Exception.h"

// 添加符号
unsigned int SymbolTable::Add(const std::string& strSymbol)
{
	m_Symbol[strSymbol] = m_uId;     // 靠字符串来查找对应ID
	return m_uId++;
}

// 查找符号
unsigned int SymbolTable::Find(const std::string& strSymbol) const
{
	std::map<const std::string, unsigned int>::const_iterator const_iter = m_Symbol.find(strSymbol);
	return (const_iter != m_Symbol.end()) ? const_iter->second : IDNOTFOUND;
}

// 清空变量
void SymbolTable::Clear()
{
	m_Symbol.clear();
	m_uId = 0;
}

// 函数对象
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

// 查找符号
std::string SymbolTable::GetSymbolName(unsigned int id)	const
{
	std::map<const std::string, unsigned int>::const_iterator const_iter;
	const_iter = find_if(m_Symbol.begin(), m_Symbol.end(), IsEqualId(id));
	if (const_iter == m_Symbol.end())
		throw Exception("Internal error: missing entry in symboltable."); //抛出异常
	return const_iter->first;
}

// 返回符号表中的函数和变量总数
unsigned int SymbolTable::GetSymtblSize() const
{
	return m_uId;
}

// 序列化方法保存变量
void SymbolTable::Serialize(Serializer& out) const
{
	out << m_Symbol.size();			//先把变量个数写入
	auto it = m_Symbol.begin();
	for (; it != m_Symbol.end(); ++it)
		out << it->first << it->second;
	out << m_uId;
}

// 反序列化方法保存数据
void SymbolTable::DeSerialize(DeSerializer& in)
{
	m_Symbol.clear();				// 先清除变量，再从文件加载
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