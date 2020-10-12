#include <iostream>
#include <cassert>
#include <sstream>
#include "Parser.h"
#include "Scanner.h"
#include "Node.h"
#include "Calc.h"
#include "Exception.h"
#include "DebugNew.h"

Parser::Parser(Scanner& scanner, Calc& calc)
	:m_Scanner(scanner), m_Calc(calc), m_Tree(0), m_Status(STATUS_OK)
{

}
Parser::~Parser()
{

}

//�������ʽ
EStatus Parser::Parse()
{
	m_Tree = Expression();
	if (!m_Scanner.IsDone()) //�ж��Ƿ�������������ʽ
	{
		m_Status = STATUS_ERROR;
	}
	return m_Status;
}


//������ʽ��ֵ
double Parser::Calculate() const
{
	assert(m_Tree.get() != 0);			//��Ϊ����ָ�벻�ܵ���0,���Բ��ܸ�0�Ƚ�,������ԭ��ָ��ſ���
	return m_Tree->Calc();				//������ֵ�����������ʽ��ֵ
}

//���ʽ
std::auto_ptr<Node> Parser::Expression()
{
	//һ�����ʽ��������+(-)���ʽ ���߾���һ���� 
	//��term���ص�ʱ��,����Դ����Ȩת�Ƹ���node����,ֻҪnode���ͷ�,�򲻻�����ڴ�й©
	std::auto_ptr<Node> node = Term(); //����һ������
	EToken token = m_Scanner.Token();//��ǰɨ�赽��״̬
	if (token == TOKEN_PLUS || token == TOKEN_MINUS)
	{
		std::auto_ptr<MultipleNode>	multipleNode(new SumNode(node));
		do
		{
			m_Scanner.ScanChar();//ɨ����һ���ַ�
			std::auto_ptr<Node> nextNode = Term();
			multipleNode->AppendChild(nextNode, (token == TOKEN_PLUS));
			token = m_Scanner.Token();//���µ�ǰ�ַ�״̬
		} while (token == TOKEN_PLUS || token == TOKEN_MINUS);
		node = multipleNode;
	}
	else if (token == TOKEN_ASSIGN)
	{
		m_Scanner.ScanChar();
		std::auto_ptr<Node> nodeRight = Expression();
		if (node->IsLvalue()) //�����������ֵ�Ļ��͸�ֵ
		{
			node = std::auto_ptr<Node>(new AssignNode(node, nodeRight));
		}
		else
		{
			m_Status = STATUS_ERROR;
			throw SyntaxError("Syntax Error: The left - hand side of an assignment must be a variable ");
		}
	}
	//�������ֻ��һ����,��ֱ�ӷ���node
	return node;
}

//��
std::auto_ptr<Node> Parser::Term()
{
	//һ��������� ��ʽ *(/) �� ���߾���һ����ʽ
	std::auto_ptr<Node> node = Factor();
	EToken token = m_Scanner.Token();			//��ǰɨ�赽��״̬

	if (token == TOKEN_MULTIPLY || token == TOKEN_DIVIDE)
	{
		std::auto_ptr<MultipleNode>	multipleNode(new ProductNode(node));
		do
		{
			m_Scanner.ScanChar();				//ɨ����һ���ַ�
			std::auto_ptr<Node> nextNode = Factor();
			multipleNode->AppendChild(nextNode, (token == TOKEN_MULTIPLY));
			token = m_Scanner.Token();			//���µ�ǰ�ַ�״̬
		} while (token == TOKEN_MULTIPLY || token == TOKEN_DIVIDE);
		node = multipleNode;
	}
	//�������ֻ��һ����ʽ,��ֱ�ӷ���node
	return node;
}
//��ʽ
std::auto_ptr<Node> Parser::Factor()
{
	//һ����ʽ������ ���� ��ʶ�� ���� (���ʽ)
	std::auto_ptr<Node> node;
	EToken token = m_Scanner.Token();//��ǰɨ�赽��״̬

	if (token == TOKEN_LPARENTHESIS)
	{
		m_Scanner.ScanChar();//ɨ�赽 (
		node = Expression();//�ݹ����
		if (m_Scanner.Token() == TOKEN_RPARENTHESIS)
		{
			m_Scanner.ScanChar();// ɨ�赽)
		}
		else
		{
			m_Status = STATUS_ERROR;
			throw SyntaxError("Syntax Error: miss ')'!");
		}
	}
	else if (token == TOKEN_IDENTIFIER)
	{
		std::string symbol = m_Scanner.GetSymbol();	 //�ӷ��ű�(������)�л�ȡ����
		unsigned int id = m_Calc.FindSymbol(symbol); //�ҵ��ñ�����ID
		m_Scanner.ScanChar();

		if (m_Scanner.Token() == TOKEN_LPARENTHESIS) //�����ʶ��������������Ǻ�������(A())
		{
			m_Scanner.ScanChar();
			node = Expression();		 //�����������Ҳ��һ�����ʽ(A(2+3))
			if (m_Scanner.Token() == TOKEN_RPARENTHESIS)
			{
				m_Scanner.ScanChar();
				if (id != SymbolTable::IDNOTFOUND && m_Calc.IsFunction(id))
				{
					node = std::auto_ptr<Node>(new FunctionNode(node, m_Calc.GetFunction(id)));
				}
				else
				{
					m_Status = STATUS_ERROR;
					std::ostringstream oss;	 //����ַ�����
					oss << "Syntax Error: Unknow function \"" << symbol << "\"";
					throw SyntaxError(oss.str());
				}
			}
			else
			{
				m_Status = STATUS_ERROR;
				throw SyntaxError("Syntax Error: miss ')' in a function call.");
			}

		}
		else
		{
			if (id == SymbolTable::IDNOTFOUND)			 //���������û�д�ŵ�storage���Ž�ȥ
			{
				id = m_Calc.AddSymbol(symbol);  //�����¼��������ID
			}
			node = std::auto_ptr<Node>(new VariableNode(id, m_Calc.GetStorage()));
		}

	}
	else if (token == TOKEN_NUMBER)
	{
		node = std::auto_ptr<Node>(new NumberNode(m_Scanner.Number()));
		m_Scanner.ScanChar();
	}
	else if (token == TOKEN_MINUS)
	{
		m_Scanner.ScanChar();
		node = std::auto_ptr<Node>(new UminusNode(Factor()));
	}
	else
	{
		m_Status = STATUS_ERROR;
		throw SyntaxError("Syntax Error: not a valid expression.");
	}
	return node;
}