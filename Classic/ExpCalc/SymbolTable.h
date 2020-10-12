#pragma once

#include <map>
#include <string>
#include "Serial.h"

class SymbolTable
{
public:
	enum{ IDNOTFOUND = 0xffffffff };
	SymbolTable() : m_uId(0) {}
	void Serialize(Serializer& out) const;						// 序列化，把类数据写入文件
	void DeSerialize(DeSerializer& in);							// 序列化，把文件数据写入类
	unsigned int Add(const std::string& strSymbol);				// 添加符号
	unsigned int Find(const std::string& strSymbol) const;		// 查找符号
	void Clear();
	std::string GetSymbolName(unsigned int id) const;			// 获取变量名
	unsigned int GetSymtblSize() const;							// 获取符号个数
private:
	std::map<std::string, unsigned int > m_Symbol;
	unsigned int m_uId;
};