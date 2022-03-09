#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <regex>
#include <sstream>

using namespace std;

class Utility
{		
public:
	void SplitString(const string& s, vector<string>& v, const string& c);

	//*****************************************************
	// Function:	 CheckData
	// FullName:	 
	// Description:	 输入值是否和数据类型匹配
	// Parameters:
	// 		const std::string strType : 输入值的数据类型
	// 		const std::string : 输入值
	// Return value: 如果匹配，返回true；如果不匹配，返回false
	// Remarks:		 
	//*****************************************************
	bool CheckData(const std::string strType, const std::string &strValue);
};