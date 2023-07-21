#include <string>
#include <iostream>

class Solution {
public:
	int LengthOfLastWord(std::string strWord)
	{
		int nIdx = strWord.size() - 1;

		while (strWord[nIdx] == ' ')
		{
			nIdx--;
		}

		int nWordLength = 0;
		while (nIdx >= 0 && strWord[nIdx] != ' ')
		{
			nWordLength++;
			nIdx--;
		}
		return nWordLength;
	}
};


int main()
{
	std::string str = "hello nowcoder ";

	Solution solu;
	int nLength = solu.LengthOfLastWord(str);
	
	std::cout << "last word length = " << nLength << std::endl;
	
	return 0;
}