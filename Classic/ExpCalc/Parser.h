#pragma once

#include <memory>

// ��������״̬
enum EStatus
{
	STATUS_OK,
	STATUS_QUIT,
	STATUS_ERROR
};

// ǰ������
class Calc;
class Node;
class Scanner;

// �������ʽ��
class Parser
{
public:
	Parser(Scanner& scanner, Calc& calc);
	~Parser();
	EStatus Parse();					// �������ʽ�������ɱ��ʽ��
	double Calculate() const;			// ������
	std::auto_ptr<Node> Expression();	// ���Ϊ���ʽ
	std::auto_ptr<Node> Term();			// ���Ϊ��
	std::auto_ptr<Node> Factor();		// ���Ϊ����
private:
	Scanner& m_Scanner;
	Calc& m_Calc;
	std::auto_ptr<Node> m_Tree;			// ���ʽ��
	EStatus m_Status;

};