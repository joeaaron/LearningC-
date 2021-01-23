#include <iostream>
#include <string>
#include "Parser.h"
#include "Calc.h"
#include "Scanner.h"
#include "Exception.h"
#include "CommandParser.h"
#include "DebugNew.h"

int main()
{
	EStatus Status = STATUS_OK;
	Calc calc;

	do
	{
		std::cout << ">>";
		Scanner scanner(std::cin);//扫描表达式
		if (!scanner.IsEmpty())	 //如果表达式不为空
		{
			if (scanner.IsCommand())	//判断是否是命令
			{
				try
				{
					CommandParser cmdparser(scanner, calc);
					Status = cmdparser.Execute();	 //执行命令		
				}
				catch (SyntaxError& se)
				{
					std::cout << se.what() << std::endl;
				}
			}
			else
			{
				Parser parser(scanner, calc);
				try
				{
					parser.Parse();//如果这个步骤出错,则直接进到异常处理,不会执行下一步了
					std::cout << parser.Calculate() << std::endl << std::endl; //计算并输出结果
				}
				catch (SyntaxError& se)//语法异常
				{
					std::cout << se.what() << std::endl;
				}
				catch (Exception& e)
				{
					std::cout << e.what() << std::endl;
				}
				catch (std::bad_alloc& e) //内存分配失败异常
				{
					std::cout << e.what() << std::endl;
				}
				catch (...)
				{
					std::cout << "Internal error." << std::endl;
				}
			}
		}
		else
			continue;
	} while (Status != STATUS_QUIT);

	return 0;
}