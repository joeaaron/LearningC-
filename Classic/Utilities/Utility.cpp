#include "Utility.h"

#define SHORT_MIN       (-32768)
#define SHORT_MAX       32767
#define USHORT_MAX      0xffff

void Utility::SplitString(const string& s, vector<string>& v, const string& c)
{
	string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while (string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2 - pos1));

		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
		v.push_back(s.substr(pos1));
}

bool Utility::CheckData(const std::string strType, const std::string &strValue)
{
	if (strType.empty())
	{
		return false;
	}

	// 将字符串转换为需要的数据类型
	std::string strRegCommandBool;		    // 正则表达式Bool型限定格式
	std::string strRegCommandWholeNum;  	// 正则表达式整型限定格式
	std::string strRegCommandDecimal;		// 正则表达式非整型限定格式

	strRegCommandBool = "^(true|false)$";
	strRegCommandWholeNum = "^(\\+|\\-)*(\\d+)$";
	strRegCommandDecimal = "^(\\+|\\-)*(\\d+)(\\.*)(\\d*)$";

	/**************************************
	//	使用正则表达式检查初步的数据格式
	***************************************/
	std::string strRawData = strValue;	// 原始变量
	std::string strCommand("");								// 匹配规则
	std::smatch matchResult;								// 匹配结果
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
		return false;	// 初步合法性检查失败
	}

	/**************************************
	//	对数值做进一步检查
	***************************************/
	std::stringstream ss;
	ss << strValue;
	double dValue = 0;
	ss >> dValue;

	long long nValue = -1;
	ss >> nValue;

	if ("float" == strType)
	{
		// Float的有效位为6~7个，超过会自动截断。 - 是否提示
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
			// 先判断字符串长度 -- 由第一个不为零的数字开始截断
			std::string strRawData = strValue;						// 原始变量
			std::string strCommand("^((\\+|\\-)*)((0)*)(\\d+?)$");	// 匹配规则
			std::smatch matchResult;								// 匹配结果
			std::string::const_iterator iterBegin = strRawData.begin();
			std::string::const_iterator iterEnd = strRawData.end();
			std::regex patternHead(strCommand, std::regex_constants::ECMAScript);
			if (!std::regex_match(iterBegin, iterEnd, matchResult, patternHead))
			{
				return false;
			}

			// 取出最后一个子字符串的匹配结果
			std::string strMatchRealNum;	// 截断前面0，不包含符号部分的数值
			if (matchResult.size())
			{
				strMatchRealNum = matchResult[matchResult.size() - 1].str().c_str();
			}

			// 判断非零部分的长度是否已经超过INT64_MAX/INT64_MIN的长度(19)
			if (strMatchRealNum.size() > 19)
				return false;
			else if (strMatchRealNum.size() < 19)
				return true;

			// 如果长度恰好等于19，由高位向低位比较
			// INT64_MAX：9223372036854775807
			// INT64_MIN：-9223372036854775808			
			bool bIsPositive(true);	// 判断数值正负

			char strSymbol = strValue.at(0);
			if (strSymbol == '-')
				bIsPositive = false;
			std::string strCompare;	// 比较数值
			if (bIsPositive)	// ＋
			{
				strCompare = "9223372036854775807";
			}
			else // －
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