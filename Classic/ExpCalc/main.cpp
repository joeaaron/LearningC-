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
		Scanner scanner(std::cin);//ɨ����ʽ
		if (!scanner.IsEmpty())	 //������ʽ��Ϊ��
		{
			if (scanner.IsCommand())	//�ж��Ƿ�������
			{
				try
				{
					CommandParser cmdparser(scanner, calc);
					Status = cmdparser.Execute();	 //ִ������		
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
					parser.Parse();//�������������,��ֱ�ӽ����쳣����,����ִ����һ����
					std::cout << parser.Calculate() << std::endl << std::endl; //���㲢������
				}
				catch (SyntaxError& se)//�﷨�쳣
				{
					std::cout << se.what() << std::endl;
				}
				catch (Exception& e)
				{
					std::cout << e.what() << std::endl;
				}
				catch (std::bad_alloc& e) //�ڴ����ʧ���쳣
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