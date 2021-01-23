#pragma once

#include <string>
#include "SymbolTable.h"
#include "Storage.h"
#include "FunctionTable.h"
#include "Serial.h"

class Parser;

class Calc
{
	//友元类,使	Parser类方便访问Calc的私有成员
	friend class Parser;
public:
	Calc() :m_Funtbl(m_Symtable), m_Storage(m_Symtable) {}
	void Serialize(Serializer& out);
	void DeSerialize(DeSerializer& in);
	void ListFunction() const;
	void ListVarible() const;
	void Clear(); //清除变量
private:
	Storage& GetStorage();				  //获取存储变量表
	bool IsFunction(unsigned int id) const;//判断是否是预定义的函数
	PtrFun GetFunction(unsigned int id) const; //获取函数
	unsigned int FindSymbol(const std::string& strSymbol) const; //查找符号
	unsigned int AddSymbol(const std::string& strSymbol);		 //增加符号
	SymbolTable m_Symtable;	 //因为不是引用,所以不能用前置声明,要包含头文件
	FunctionTable m_Funtbl;	 //构造的次序与这里的定义顺序有关,与初始化列表无关
	Storage m_Storage;
};




