#include <string>
#include <iostream>
#include <algorithm>
#include <unordered_map>
using namespace std;

class Solution {
public:
	int lengthOfLonestSubstring(string s){
		size_t ret = 0, start = 0;
		unordered_map<char, size_t> trace;        //来边记录边比较(max)
		for (size_t i = 0; i < s.size(); ++i){
			auto found = trace.find(s[i]);
			if (found != trace.end() && found->second >= start){
				ret = max(ret, i - start);
				start = found->second + 1;
			}
			trace[s[i]] = i;
		}
		return max(ret, s.size() - start);
	}
};


int main()
{
	string str = "abcabcdb";
	Solution solu;
	int length = solu.lengthOfLonestSubstring(str);
	cout << "the max length = " << length << endl;
	system("pause");
	return 0;
}