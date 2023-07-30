#include "CompareVersion.h"

std::vector<int> ComapreVersion::SplitVersion(const std::string& strVersion)
{
	std::vector<int> vResult;	

	if (!strVersion.empty())
	{
		size_t pos(0), begin(0);
		int nVersionPer;
		while (pos != std::string::npos)
		{
			pos = strVersion.find('.', begin);
			if (pos == std::string::npos)
			{
				try
				{
					nVersionPer = std::stoi(strVersion.substr(begin));
					vResult.push_back(nVersionPer);
				}
				catch (const std::invalid_argument& e)	// ���� std::invalid_argument �쳣
				{								
					std::cerr << e.what() << std::endl;
					vResult.push_back(0);
				}
				catch (const std::out_of_range& e)		// ���� std::out_of_range �쳣
				{								
					std::cerr << e.what() << std::endl;
					vResult.push_back(0);
				}
					
				break;
			}
			nVersionPer = std::stoi(strVersion.substr(begin, pos - begin));
			vResult.push_back(nVersionPer);
			begin = pos + sizeof('.');
		}
	}
	return vResult;
}

int ComapreVersion::CompareVersion(const std::string& strCurVersion, const std::string& strHisVersion)
{
	std::vector<int> vHisVersion = SplitVersion(strHisVersion);
	std::vector<int> vCurVersion = SplitVersion(strCurVersion);
	int nCompResult = 0;

	size_t versionsize = vCurVersion.size() < vHisVersion.size() ? vCurVersion.size() : vHisVersion.size();

	for (size_t i = 0; i < versionsize; ++i)
	{
		if (vCurVersion[i] == vHisVersion[i])
		{
			if (i == vCurVersion.size() - 1)
			{
				nCompResult = 0;      				  // �汾��ͬ
			}
			else
			{
				continue;
			}
		}
		if (vCurVersion[i] > vHisVersion[i])		  // ��ǰ�汾>��ʷ�汾
		{
			nCompResult = 1;
			break;
		}
		if (vCurVersion[i] < vHisVersion[i])          // ��ǰ�汾<��ʷ�汾
		{
			nCompResult = -1;
			break;
		}
	}

	return nCompResult;
}