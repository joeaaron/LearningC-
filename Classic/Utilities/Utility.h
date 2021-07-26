#pragma once

#include <vector>
#include <string>
#include <iostream>

using namespace std;

class Utility
{
public:
	Utility();   
	~Utility();			
public:
	void SplitString(const string& s, vector<string>& v, const string& c);


};