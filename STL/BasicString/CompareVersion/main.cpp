
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
		std::cout << "�汾����ͬ!" << std::endl;
	}
	else if (1 == nResult)
	{
		std::cout << "��ǰ�汾>��ʷ�汾!" << std::endl;
	}
	else
	{
		std::cout << "��ǰ�汾<��ʷ�汾!" << std::endl;
	}
}