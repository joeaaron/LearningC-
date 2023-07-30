
#include "CompareVersion.h"

int main()
{
	std::string strVersionHis = "1.0.0.1";
	std::string strVersionCur = "1.0.0.0";

	ComapreVersion versionCmp;
	int nResult = 0;
	nResult = versionCmp.CompareVersion(strVersionCur, strVersionHis);
	if (0 == nResult)
	{
		std::cout << "版本号相同!" << std::endl;
	}
	else if (1 == nResult)
	{
		std::cout << "当前版本>历史版本!" << std::endl;
	}
	else
	{
		std::cout << "当前版本<历史版本!" << std::endl;
	}
}