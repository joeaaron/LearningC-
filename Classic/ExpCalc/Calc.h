#pragma once

#include <string>
#include "SymbolTable.h"
#include "Storage.h"
#include "FunctionTable.h"
#include "Serial.h"

class Parser;

class Calc
{
	//��Ԫ��,ʹ	Parser�෽�����Calc��˽�г�Ա
	friend class Parser;
public:
	Calc() :m_Funtbl(m_Symtable), m_Storage(m_Symtable) {}
	void Serialize(Serializer& out);
	void DeSerialize(DeSerializer& in);
	void ListFunction() const;
	void ListVarible() const;
	void Clear(); //�������
private:
	Storage& GetStorage();				  //��ȡ�洢������
	bool IsFunction(unsigned int id) const;//�ж��Ƿ���Ԥ����ĺ���
	PtrFun GetFunction(unsigned int id) const; //��ȡ����
	unsigned int FindSymbol(const std::string& strSymbol) const; //���ҷ���
	unsigned int AddSymbol(const std::string& strSymbol);		 //���ӷ���
	SymbolTable m_Symtable;	 //��Ϊ��������,���Բ�����ǰ������,Ҫ����ͷ�ļ�
	FunctionTable m_Funtbl;	 //����Ĵ���������Ķ���˳���й�,���ʼ���б��޹�
	Storage m_Storage;
};




