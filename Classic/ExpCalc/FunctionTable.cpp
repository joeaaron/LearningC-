#include <cmath>
#include <cassert>

#include "FunctionTable.h"
#include "SymbolTable.h"

struct FunctionEntry
{
	PtrFun pFun;
	char* pFunName;
};

FunctionEntry Entrys[] =
{
	log, "log",
	log10, "log10",
	exp, "exp",
	sqrt, "sqrt",
	sin, "sin",
	cos, "cos",
	tan, "tan",
	sinh, "sinh",
	cosh, "cosh",
	tanh, "tanh",
	asin, "asin",
	acos, "acos",
	atan, "atan"
};

FunctionTable::FunctionTable(SymbolTable& symtbl)
	:m_Size(sizeof(Entrys) / sizeof(Entrys[0]))
{
	Init(symtbl);
}

// 初始化
void FunctionTable::Init(SymbolTable& symtbl)
{
	m_pFuns = new PtrFun[m_Size];
	for (int i = 0; i < m_Size;i++)
	{
		m_pFuns[i] = Entrys[i].pFun;
		unsigned int j = symtbl.Add(Entrys[i].pFunName);		// 把函数加入符号表
		assert(i == j);
	}
}
FunctionTable::~FunctionTable()
{
	delete[] m_pFuns;
}

unsigned int FunctionTable::GetSize() const
{
	return m_Size;
}