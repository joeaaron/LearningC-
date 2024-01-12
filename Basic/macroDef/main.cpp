#include "macroDef.h"
#include <iostream>

BEGIN_REG_COMMAND_FUN
REG_COMMAND_FUN(0, IsOnline)

END_REG_COMMAND_FUN

bool IsOnline()
{
	std::cout << "no device online.";
	return false;
}

bool CommandService(const char* strCommandName, HParamCam commandParam = nullptr)
{
	bool bRet = true; 

	switch (GET_COMMAND_ID(strCommandName))
	{
	case 0:
	{
		bRet = IsOnline();
	}
	break;

	default:
		break;
	}

	return bRet;
}

int main()
{
	 bool bOnline = CommandService("IsOnline");

	 return 0;
}