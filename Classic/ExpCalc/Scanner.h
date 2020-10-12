#pragma once

// ��ʶɨ�赽����ʲô�ַ�
enum EToken
{
	TOKEN_END,					// ��ֹ�ַ�
	TOKEN_ERROR,				// ɨ�赽�����ַ�
	TOKEN_NUMBER,				// ���� 
	TOKEN_PLUS,					// +
	TOKEN_MINUS,				// -
	TOKEN_MULTIPLY,				// *
	TOKEN_DIVIDE,				// /
	TOKEN_LPARENTHESIS,			// (
	TOKEN_RPARENTHESIS,			// )
	TOKEN_IDENTIFIER,			// ��ʶ��
	TOKEN_ASSIGN,				// ��ֵ��
	TOKEN_COMMAND				// ����
};

// ɨ����ʽ��
class Scanner
{
public:
	explicit Scanner(std::istream& in);	     // ��ֹ��ʽת��
	void ScanChar();						 // ɨ��һ���ַ�
	double Number() const;					 // ��������
	EToken Token() const;					 // ����ɨ�赽���ַ�����
	std::string GetSymbol() const;			 // ��ȡ��ʶ��
	bool IsEmpty() const;					 // �жϱ��ʽ�Ƿ�Ϊ��
	bool IsDone() const;					 // �ж��Ƿ�ɨ�����������ʽ
	bool IsCommand() const;					 // �ж��Ƿ�������
	void GetCmdParam();						 // ��ȡ�������

private:
	void ReadChar();						 // ����������һ�����ַ�����
	std::istream& m_In;						 // ������
	double m_Number;						 // ����
	EToken m_Token;							 // ��ʶɨ�赽����ʲô�ַ�
	std::string m_SymBol;					 // ������ʶ��
	bool m_bIsEmpty;						 // �жϱ��ʽ�Ƿ�Ϊ��
	int m_Look;								 // ��ǰ���������ַ�
};