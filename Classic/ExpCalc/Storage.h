#pragma once

#include <vector>
#include "Serial.h"

class SymbolTable;
class Storage
{
public:
	Storage(SymbolTable& symtable);
	void Serialize(Serializer& out) const;	       //���л�����������д���ļ�
	void DeSerialize(DeSerializer& in);			   //���л������ļ�����д����
	void Clear();							       //��ձ�������
	void AddConstants(SymbolTable& symtable);	   //���ӳ���
	bool IsInit(unsigned int id) const;			   //�жϱ����Ƿ��ʼ��
	double GetValue(unsigned int id) const;		   //��ȡ�������߳�����ֵ
	void SetValue(unsigned int id, double value);  //���ó����������ֵ
	void AddValue(unsigned int id, double value);  //��ӳ����������ֵ

private:
	std::vector<double> m_Cells;					//��������ͳ���
	std::vector<bool> m_Inits;						//�����Ƿ��Ѿ���ʼ��
};