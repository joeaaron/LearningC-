#include "Utility.h"

int main()
{
	std::string strType = "int";
	std::string strVal = "0.123123";

	Utility util;

	bool bValid = util.CheckData(strType, strVal);
	
	std::cout << bValid;


	return 0;
}

