#pragma once

class SymbolTable;
// 函数指针
typedef double(*PtrFun)(double);

class FunctionTable
{
public:
	FunctionTable(SymbolTable& symtbl);
	~FunctionTable();

	void Init(SymbolTable& symtbl);					// 初始化函数指针数组
	PtrFun GetFunction(unsigned int id) const { return m_pFuns[id]; }
	unsigned int Size() const{ return m_Size; }		// 有几个函数
	unsigned int GetSize() const;
private:
	PtrFun* m_pFuns;
	unsigned int m_Size;

};

