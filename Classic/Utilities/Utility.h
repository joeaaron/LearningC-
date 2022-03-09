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
	// Description:	 ����ֵ�Ƿ����������ƥ��
	// Parameters:
	// 		const std::string strType : ����ֵ����������
	// 		const std::string : ����ֵ
	// Return value: ���ƥ�䣬����true�������ƥ�䣬����false
	// Remarks:		 
	//*****************************************************
	bool CheckData(const std::string strType, const std::string &strValue);
};