#pragma once

class SymbolTable;
// ����ָ��
typedef double(*PtrFun)(double);

class FunctionTable
{
public:
	FunctionTable(SymbolTable& symtbl);
	~FunctionTable();

	void Init(SymbolTable& symtbl);					// ��ʼ������ָ������
	PtrFun GetFunction(unsigned int id) const { return m_pFuns[id]; }
	unsigned int Size() const{ return m_Size; }		// �м�������
	unsigned int GetSize() const;
private:
	PtrFun* m_pFuns;
	unsigned int m_Size;

};

