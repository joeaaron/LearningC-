#include <vector>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

using namespace std;

class Solution {
public:
	template<typename T>
	vector<int> FindPair(const T arr[], int n, const T weightDiff)
	{
		for (int i = 0; i < n; ++i)
		{
			for (int j = i + 1; j< n; ++j)
			{
				if (arr[j] == weightDiff + arr[i] || arr[i] == weightDiff + arr[j])
				{
					return { i , j };
					break;
				}
			}
		}

		return { -1, -1 };
	}

	template<typename T>
	vector<int> FindPairEx(const T arr[], int n, const T weightDiff)
	{
		if (n < 1 || n > pow(10, 4))
		{
			cout << "The num of stones not in range!";
			return { -1, -1 };
		}

		if (weightDiff < -pow(10, 7) || weightDiff > pow(10, 7))
		{
			cout << "The weight difference not in range!";
			return { -1, -1 };
		}

		std::unordered_map<int, int> record;
		for (int i = 0; i != n; ++i)
		{
			if (arr[i] < -pow(10, 7) || arr[i] > pow(10, 7))
			{
				cout << "The weight of stone not in range!";
				continue;
			}

			auto found = record.find(arr[i]);

			if (found != record.end())
			{
				cout << "(" << arr[found->second] << ", " << arr[i] << ")\n";
				return{ found->second, i };
				
			}
				   
			record.emplace(weightDiff + arr[i], i);
		}

		return{ -1, -1 };
	}

	template<typename T>
	int FindPairs(T *arr, int n, const T weightDiff)
	{
		unordered_map<T, int> freq;

		int pairCount = 0;

		for (int i = 0; i < n; ++i)
		{
			T complement = arr[i] + weightDiff;
			pairCount += freq[complement];

			if (weightDiff != 0)
			{
				complement = arr[i] - weightDiff;
				pairCount += freq[complement];
			}

			++freq[arr[i]];
		}

		return pairCount;
	}

	template<typename T>
	void FindPairsEx(T arr[], int n, const T weightDiff)
	{
		if (n < 1 || n > pow(10, 4))
		{
			cout << "The num of stones not in range!";
			throw invalid_argument("the input has no solution");
		}

		if (weightDiff < -pow(10, 7) || weightDiff > pow(10, 7))
		{
			cout << "The weight difference not in range!";
			throw invalid_argument("the input has no solution");
		}

		// sort array in ascending order
		sort(arr, arr + n);

		// take an empty set
		unordered_set<T> set;

		// do for every array element
		for (int i = 0; i < n; i++)
		{
			if (arr[i] < -pow(10, 7) || arr[i] > pow(10, 7))
			{
				cout << "The weight of stone not in range!";
				continue;
			}


			// to avoid printing duplicates (skip adjacent duplicates)
			while (i + 1 < n && arr[i] == arr[i + 1]) 
			{
				i++;
			}

			// check if pair with the given difference `(arr[i], arr[i]-weightDiff)` exists
			if (set.find(arr[i] - weightDiff) != set.end()) 
			{
				cout << "(" << arr[i] << ", " << arr[i] - weightDiff << ")\n";
			}

			// check if pair with the given difference `(arr[i]+weightDiff, arr[i])` exists
			if (set.find(arr[i] + weightDiff) != set.end()) 
			{
				cout << "(" << arr[i] + weightDiff << ", " << arr[i] << ")\n";
			}

			// insert the current element into the set
			set.insert(arr[i]);
		}
	}
};


int main()
{
	/*vector<double> vec ={7, 4, 4, 5, 6, 7, 5,10};
	double tar = 3;
	Solution solu;

	vector<int> p = solu.FindPairEx(vec, tar);
	for (auto &num : p)
		cout << num << endl;*/

	double arr[] = { 0, 3, 6, 5, 3, 9 };
	double diff = -3;

	int n = sizeof(arr) / sizeof(arr[0]);

	Solution solu;
	cout << solu.FindPairs(arr, n, diff);


	system("pause");
	return 0;
}