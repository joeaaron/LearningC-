#pragma once
#include <vector>
#include <string>
#include <iostream>

class ComapreVersion
{
public:
	ComapreVersion(){}
	~ComapreVersion() {}

public:
	std::vector<int> SplitVersion(const std::string& strVersion);
	int CompareVersion(const std::string& strCurVersion, const std::string& strHisVersion);
};

