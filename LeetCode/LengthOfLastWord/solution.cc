#include <string>
#include <iostream>

using namespace std;

class Solution {
public:
	int lengthOfLastWord(string s)
	{
		int nIdx = s.size() - 1;

		while (s[nIdx] == ' ')
		{
			nIdx--;
		}

		int nWordLength = 0;
		while (nIdx >= 0 && s[nIdx] != ' ')
		{
			nWordLength++;
			nIdx--;
		}
		return nWordLength;
	}
};


int main()
{
	string str = "hello aw";

	Solution solu;
	int nLength = solu.lengthOfLastWord(str);
	
	cout << "last word length = " << nLength << endl;
	
	return 0;
}