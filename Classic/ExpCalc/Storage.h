#pragma once

#include <vector>
#include "Serial.h"

class SymbolTable;
class Storage
{
public:
	Storage(SymbolTable& symtable);
	void Serialize(Serializer& out) const;	       //序列化，把类数据写入文件
	void DeSerialize(DeSerializer& in);			   //序列化，把文件数据写入类
	void Clear();							       //清空变量常量
	void AddConstants(SymbolTable& symtable);	   //增加常量
	bool IsInit(unsigned int id) const;			   //判断变量是否初始化
	double GetValue(unsigned int id) const;		   //获取变量或者常量的值
	void SetValue(unsigned int id, double value);  //设置常量或变量的值
	void AddValue(unsigned int id, double value);  //添加常量或变量的值

private:
	std::vector<double> m_Cells;					//保存变量和常量
	std::vector<bool> m_Inits;						//变量是否已经初始化
};