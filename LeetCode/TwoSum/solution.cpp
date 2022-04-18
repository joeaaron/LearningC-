#include <vector>
#include <iostream>
#include <unordered_map>
using namespace std;

class Solution {
public:
	template<typename T>
	vector<int> twoSum(vector<T>& nums, int target) {
		std::unordered_map<int, int> record;
		for (int i = 0; i != nums.size(); ++i) {
			auto found = record.find(nums[i]);
			if (found != record.end())  
				return{ found->second, i };        //£¨11£¬0£©£¨8£¬1£©£¨6£¬2£©£¨5£¬3£©
			record.emplace(target - nums[i], i);
		}
		return{ -1, -1 };
	}

	template<typename T>
	int findPairs(vector<T>& nums, int k) {
		int res = 0;
		unordered_map<int, int> m;
		for (int num : nums) ++m[num];
		for (auto a : m) {
			if (k == 0 && a.second > 1) ++res; // multiple duplicate
			if (k > 0 && m.count(a.first + k)) ++res;
		}
		return res;
	}
};


int main()
{
	vector<double> vec = { 1, 4.4, 6, 7, 8.8, 13 };
	int tar = 4;
	Solution solu;
	//vector<int>  p = solu.twoSum(vec, tar);  //find the index
	//for (auto &num : p)
	//	cout << num << endl;

	int res = solu.findPairs(vec, tar);
	cout << res << endl;

	system("pause");
	return 0;
}