#include <iostream>
#include "Scanner.h"

Scanner::Scanner(std::istream& in)
	:m_In(in)
{
	ScanChar();								// �����������ʱ�����ɨ��һ���ַ�
	m_bIsEmpty = (m_Token == TOKEN_END);	// ��һ��ɨ�������س����ǿյ�
}

// һ������ȡ�ַ�
void Scanner::ReadChar()  
{
	m_Look = m_In.get();					// �ӱ�׼�����л�ȡ�ַ�
	while (m_Look == ' ' || m_Look == '\t')	// ���Կո�
		m_Look = m_In.get();
}

// �ж��Ƿ�������
bool Scanner::IsCommand() const
{
	return m_Token == TOKEN_COMMAND;
}

// ɨ��һ��һ���ַ�
void Scanner::ScanChar()
{
	// �����ַ�
	ReadChar();
	switch (m_Look)
	{
	case '#':
		m_Token = TOKEN_COMMAND;
		break;
	case '+':
		m_Token = TOKEN_PLUS;
		break;
	case '-':
		m_Token = TOKEN_MINUS;
		break;
	case '*':
		m_Token = TOKEN_MULTIPLY;
		break;
	case '/':
		m_Token = TOKEN_DIVIDE;
		break;
	case '=':
		m_Token = TOKEN_ASSIGN;
		break;
	case '(':
		m_Token = TOKEN_LPARENTHESIS;
		break;
	case ')':
		m_Token = TOKEN_RPARENTHESIS;
		break;
	case '0':case'1': case '2':case'3':case '4':
	case'5':case '6':case'7':case '8':case'9':
	case '.':
		m_Token = TOKEN_NUMBER;
		m_In.putback(m_Look);		// �Ѷ�ȡ�ķŻ�ȥ
		m_In >> m_Number;			// �������������ַŵ�number��
		break;
	case '\0':case'\n':case '\r':case EOF:
		m_Token = TOKEN_END;//����������ַ����ʾ��������
		break;
	default:
		// ������������ĸ�����»��߿�ͷ
		if (isalpha(m_Look) || m_Look == '_')
		{
			m_Token = TOKEN_IDENTIFIER;
			m_SymBol.erase();		//�Ȱ���ʱ�������,�ٻ�ȡ�±���
			// ��ȡ������
			do
			{
				m_SymBol += m_Look;
				m_Look = m_In.get();
			} while (isalnum(m_Look) || m_Look == '_');
			m_In.putback(m_Look);//�������ȡ���ַ��Ż�����
		}
		else
			m_Token = TOKEN_ERROR;
		break;
	}
}
 
// ��������
double Scanner::Number() const
{
	return m_Number;
}

// ����ɨ��״̬
EToken Scanner::Token() const
{
	return m_Token;
}

// ��ȡ��ʶ��
std::string Scanner::GetSymbol() const
{
	return m_SymBol;
}

// �жϱ��ʽ�Ƿ�Ϊ��
bool Scanner::IsEmpty() const
{
	return m_bIsEmpty;
}

// �жϱ��ʽ�Ƿ�ɨ����
bool Scanner::IsDone() const
{
	return m_Token == TOKEN_END;//���û��ɨ�赽������,����ʽû�б�ɨ����
}

// ��ȡ�������
void Scanner::GetCmdParam()
{
	ReadChar();
	m_SymBol.erase();
	while (!isspace(m_Look))
	{
		m_SymBol += m_Look;		//��������뵽���ű�
		m_Look = m_In.get();
	}
}