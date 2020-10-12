#pragma once
#include "Parser.h"

class Scanner;
class Calc;
class CommandParser
{
	enum ECommand
	{
		CMD_HELP,   //帮助命令
		CMD_QUIT,	//退出命令
		CMD_VAR,	//列出变量命令
		CMD_FUN,	//列出函数命令
		CMD_LOAD,	//加载文件命令
		CMD_SAVE,	//保存文件命令
		CMD_CLS,    //清屏命令
		CMD_CLEAR,  //清除变量命令
		CMD_ERROR	//错误命令
	};
public:
	CommandParser(Scanner& scanner, Calc& calc);
	EStatus Execute();
private:
	void ParserCmd(const std::string cmd);			// 解析是哪个命令
	void Help() const;								// 帮助
	void ListVar() const;		//列出变量
	void ListFun() const;		//列出函数
	void Clc() const;			//清屏
	void Clear() const;				//清除变量
	EStatus Load(const std::string& filename);
	EStatus Save(const std::string& filename);
private:
	Scanner& m_Scanner;
	Calc&    m_Calc;
	ECommand m_Cmd;
	std::string m_CmdName; //命令名称
};