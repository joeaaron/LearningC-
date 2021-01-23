#pragma once
#include "Parser.h"

class Scanner;
class Calc;
class CommandParser
{
	enum ECommand
	{
		CMD_HELP,   //��������
		CMD_QUIT,	//�˳�����
		CMD_VAR,	//�г���������
		CMD_FUN,	//�г���������
		CMD_LOAD,	//�����ļ�����
		CMD_SAVE,	//�����ļ�����
		CMD_CLS,    //��������
		CMD_CLEAR,  //�����������
		CMD_ERROR	//��������
	};
public:
	CommandParser(Scanner& scanner, Calc& calc);
	EStatus Execute();
private:
	void ParserCmd(const std::string cmd);			// �������ĸ�����
	void Help() const;								// ����
	void ListVar() const;		//�г�����
	void ListFun() const;		//�г�����
	void Clc() const;			//����
	void Clear() const;				//�������
	EStatus Load(const std::string& filename);
	EStatus Save(const std::string& filename);
private:
	Scanner& m_Scanner;
	Calc&    m_Calc;
	ECommand m_Cmd;
	std::string m_CmdName; //��������
};