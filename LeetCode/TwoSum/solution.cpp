#include <vector>
#include <iostream>
#include <unordered_map>
using namespace std;

class Solution {
public:
	vector<int> twoSum(vector<int>& nums, int target) {
		std::unordered_map<int, int> record;
		for (int i = 0; i != nums.size(); ++i) {
			auto found = record.find(nums[i]);
			if (found != record.end())  
				return{ found->second, i };        //£¨11£¬0£©£¨8£¬1£©£¨6£¬2£©£¨5£¬3£©
			record.emplace(target - nums[i], i);
		}
		return{ -1, -1 };
	}
};


int main()
{
	vector<int> vec = { 1, 4, 6, 7, 8, 13 };
	int tar = 12;
	Solution solu;
	vector<int>  p = solu.twoSum(vec, tar);  //find the index
	for (auto &num : p)
		cout << num << endl;
	system("pause");
	return 0;
}