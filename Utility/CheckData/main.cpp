#include <iostream>
#include <string>
#include <regex>
#include <sstream>

#define SHORT_MIN       (-32768)
#define SHORT_MAX       32767
#define USHORT_MAX      0xffff

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

bool CheckData(const std::string strType, const std::string &strValue)
{
	if (strType.empty())
	{
		return false;
	}

	// ���ַ���ת��Ϊ��Ҫ����������
	std::string strRegCommandBool;		    // ������ʽBool���޶���ʽ
	std::string strRegCommandWholeNum;  	// ������ʽ�����޶���ʽ
	std::string strRegCommandDecimal;		// ������ʽ�������޶���ʽ

	strRegCommandBool = "^(true|false)$";
	strRegCommandWholeNum = "^(\\+|\\-)*(\\d+)$";
	strRegCommandDecimal = "^(\\+|\\-)*(\\d+)(\\.*)(\\d*)$";

	/**************************************
	//	ʹ��������ʽ�����������ݸ�ʽ
	***************************************/
	std::string strRawData = strValue;	// ԭʼ����
	std::string strCommand("");								// ƥ�����
	std::smatch matchResult;								// ƥ����
	std::string::const_iterator iterBegin = strRawData.begin();
	std::string::const_iterator iterEnd = strRawData.end();

	if ("bool" == strType)
	{
		strCommand = std::move(strRegCommandBool);
	}
	else if ("double" == strType || "float" == strType)
	{
		strCommand = std::move(strRegCommandDecimal);
	}
	else
	{
		strCommand = std::move(strRegCommandWholeNum);
	}

	std::regex patternHead(strCommand, std::regex_constants::ECMAScript);

	if (!std::regex_search(iterBegin, iterEnd, matchResult, patternHead))
	{
		return false;	// �����Ϸ��Լ��ʧ��
	}

	/**************************************
	//	����ֵ����һ�����
	***************************************/
	std::stringstream ss;
	ss << strValue;
	double dValue = 0;
	ss >> dValue;

	long long nValue = -1;
	ss >> nValue;

	if ("float" == strType)
	{
		// Float����ЧλΪ6~7�����������Զ��ضϡ� - �Ƿ���ʾ
		if (dValue >= -3.402823466e38 && dValue <= 3.402823466e38)
			return true;
		else
			return false;
	}
	else if ("double" == strType)
	{
		if (dValue >= -1.7976931348623157e308 && dValue <= 1.7976931348623157e308)
			return true;
		else
			return false;
	}
	else
	{
		if ("char" == strType)
		{
			if (nValue >= INT8_MIN && nValue <= INT8_MAX)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else if ("unsigned char" == strType)
		{
			if (nValue >= 0 && nValue <= UINT8_MAX)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else if ("short" == strType)
		{
			if (nValue >= SHORT_MIN && nValue <= SHORT_MAX)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else if ("unsigned short" == strType)
		{
			if (nValue >= 0 && nValue <= USHORT_MAX)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else if ("long" == strType || "int" == strType)
		{
			if (nValue >= INT_MIN && nValue <= INT_MAX)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else if ("unsigned long" == strType)
		{
			if (nValue >= 0 && nValue <= ULONG_MAX)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else if ("int64" == strType)
		{
			// ���ж��ַ������� -- �ɵ�һ����Ϊ������ֿ�ʼ�ض�
			std::string strRawData = strValue;						// ԭʼ����
			std::string strCommand("^((\\+|\\-)*)((0)*)(\\d+?)$");	// ƥ�����
			std::smatch matchResult;								// ƥ����
			std::string::const_iterator iterBegin = strRawData.begin();
			std::string::const_iterator iterEnd = strRawData.end();
			std::regex patternHead(strCommand, std::regex_constants::ECMAScript);
			if (!std::regex_match(iterBegin, iterEnd, matchResult, patternHead))
			{
				return false;
			}

			// ȡ�����һ�����ַ�����ƥ����
			std::string strMatchRealNum;	// �ض�ǰ��0�����������Ų��ֵ���ֵ
			if (matchResult.size())
			{
				strMatchRealNum = matchResult[matchResult.size() - 1].str().c_str();
			}

			// �жϷ��㲿�ֵĳ����Ƿ��Ѿ�����INT64_MAX/INT64_MIN�ĳ���(19)
			if (strMatchRealNum.size() > 19)
				return false;
			else if (strMatchRealNum.size() < 19)
				return true;

			// �������ǡ�õ���19���ɸ�λ���λ�Ƚ�
			// INT64_MAX��9223372036854775807
			// INT64_MIN��-9223372036854775808			
			bool bIsPositive(true);	// �ж���ֵ����
	
			char strSymbol = strValue.at(0);
			if (strSymbol == '-')
				bIsPositive = false;
			std::string strCompare;	// �Ƚ���ֵ
			if (bIsPositive)	// ��
			{
				strCompare = "9223372036854775807";
			}
			else // ��
			{
				strCompare = "9223372036854775808";
			}
			for (unsigned int i = 0; i < strCompare.size(); i++)
			{
				if (strMatchRealNum[i] > strCompare[i])
				{
					return false;
				}
			}

			return true;
		}
	}
	return true;
}

int main()
{
	std::string strType = "int";
	std::string strVal = "123123";

	bool bValid = CheckData(strType, strVal);
	std::cout << bValid;

	return 0;
}