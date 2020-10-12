#pragma once

#include <map>
#include <string>
#include "Serial.h"

class SymbolTable
{
public:
	enum{ IDNOTFOUND = 0xffffffff };
	SymbolTable() : m_uId(0) {}
	void Serialize(Serializer& out) const;						// ���л�����������д���ļ�
	void DeSerialize(DeSerializer& in);							// ���л������ļ�����д����
	unsigned int Add(const std::string& strSymbol);				// ��ӷ���
	unsigned int Find(const std::string& strSymbol) const;		// ���ҷ���
	void Clear();
	std::string GetSymbolName(unsigned int id) const;			// ��ȡ������
	unsigned int GetSymtblSize() const;							// ��ȡ���Ÿ���
private:
	std::map<std::string, unsigned int > m_Symbol;
	unsigned int m_uId;
};