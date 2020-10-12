#include <cassert>
#include <string>
#include <sstream>
#include <iostream>
#include "CommandParser.h"
#include "Scanner.h"
#include "Calc.h"
#include "Exception.h"


CommandParser::CommandParser(Scanner& scanner, Calc& calc)
	:m_Scanner(scanner), m_Calc(calc)
{
	assert(m_Scanner.IsCommand());
	scanner.ScanChar();
	m_CmdName = scanner.GetSymbol();
	ParserCmd(m_CmdName);
}

EStatus CommandParser::Execute()
{
	EStatus status = STATUS_OK;
	m_Scanner.GetCmdParam();//获取命令参数
	switch (m_Cmd)
	{
	case CMD_HELP:
		Help();
		break;
	case CMD_QUIT:
		status = STATUS_QUIT;
		break;
	case CMD_VAR:
		ListVar();
		break;
	case CMD_FUN:
		ListFun();
		break;
	case CMD_LOAD:
		status = Load(m_Scanner.GetSymbol());
		break;
	case CMD_SAVE:
		status = Save(m_Scanner.GetSymbol());
		break;
	case CMD_CLS:
		Clc();
		break;
	case CMD_CLEAR:
		Clear();
		break;
	default:
		status = STATUS_ERROR;
		std::ostringstream oss;
		oss << "Syntax error: Unknow command \"" << m_CmdName << "\"";
		throw SyntaxError(oss.str());
		break;
	}
	return status;
}

void CommandParser::ParserCmd(const std::string cmd)
{
	if ("help" == cmd)		m_Cmd = CMD_HELP;
	else if ("quit" == cmd)	m_Cmd = CMD_QUIT;
	else if ("var" == cmd)	m_Cmd = CMD_VAR;
	else if ("fun" == cmd)	m_Cmd = CMD_FUN;
	else if ("load" == cmd)	m_Cmd = CMD_LOAD;
	else if ("save" == cmd) m_Cmd = CMD_SAVE;
	else if ("cls" == cmd)	m_Cmd = CMD_CLS;
	else if ("clear" == cmd)m_Cmd = CMD_CLEAR;
	else					m_Cmd = CMD_ERROR;

}

void CommandParser::Help() const
{
	std::cout << "Recognized commands:" << std::endl;
	std::cout << "#help" << std::endl;
	std::cout << "#quit" << std::endl;
	std::cout << "#var" << std::endl;
	std::cout << "#fun" << std::endl;
	std::cout << "#load" << std::endl;
	std::cout << "#save" << std::endl;
}
void CommandParser::ListVar() const
{
	std::cout << "varible list:" << std::endl;
	std::cout << "--------------------" << std::endl;
	m_Calc.ListVarible();
	std::cout << std::endl;
}
void CommandParser::ListFun() const
{
	std::cout << "function list:" << std::endl;
	std::cout << "--------------------" << std::endl;
	m_Calc.ListFunction();
	std::cout << std::endl;
}

void CommandParser::Clc() const
{
	system("cls");
}

void CommandParser::Clear() const
{
	m_Calc.Clear();
}

const double lVersion = 1.0; //版本号

EStatus CommandParser::Load(const std::string& filename)
{
	EStatus status = STATUS_OK;
	std::cout << "load \"" << filename << "\"" << std::endl;
	try
	{
		DeSerializer in(filename);
		double version;
		in >> version;
		if (version != lVersion)
			throw Exception("Miss macth version.");
		m_Calc.DeSerialize(in);
	}
	catch (FileStreamError& e)
	{
		std::cout << e.what() << std::endl;;
		status = STATUS_ERROR;
	}
	catch (Exception& e)
	{
		std::cout << e.what() << std::endl;;
		status = STATUS_ERROR;
	}

	return status;
}

EStatus CommandParser::Save(const std::string& filename)
{
	EStatus status = STATUS_OK;
	std::cout << "save \"" << filename << "\"" << std::endl;
	try
	{
		Serializer out(filename);
		out << lVersion;     //先把版本号写进去
		m_Calc.Serialize(out);
	}
	catch (FileStreamError& e)
	{
		std::cout << e.what() << std::endl;;
		status = STATUS_ERROR;
	}

	return status;
}