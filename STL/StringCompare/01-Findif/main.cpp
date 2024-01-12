#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

int main() 
{
	std::vector<std::string> vFruit = { "apple", "banana", "orange", "peach" };
	std::string strFruit = "banana";

	auto it = std::find_if(vFruit.begin(), vFruit.end(), [&](const std::string& str) {
		return str.find(strFruit) != std::string::npos;
	});

	if (it != vFruit.end()) 
	{
		std::cout << "Found: " << *it << std::endl;
	}
	else 
	{
		std::cout << "Not found." << std::endl;
	}
	return 0;
}