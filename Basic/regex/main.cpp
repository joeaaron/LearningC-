/*
该实例通过一个函数is_email_valid 来检查一个email地址是否是一个正确的格式。如果格式正确则返回true。
首先注意‘()’表示将正则表达式分成子表达式，每个‘()’之间的内容表示一个子表达式；
‘\’是一个转义字符，‘\\’表示扔掉第二个‘\’的转义特性，‘\w+’表示匹配一个或多个单词，‘+’表示重复一次或者多次，因此第一个子表达式的意思就是匹配一个或者多个单词；
接着看第二个子表达式，‘|’表示选择，出现‘.’或者‘_’，后面的‘?’表示该子表示出现一次或者零次，因此第二个子表示表示‘.’或‘_’出现不出现都匹配。
第三个子表达式表示出现一个单词，‘*’表示任意个字符。后面的子表示根据已经介绍的内容，已经可以容易理解，就不再赘述。
通过对正则表达式匹配模式串的分析，可以容易理解运行结果.
*/
#include <iostream>
#include <regex>
#include <map>

bool IsEmailValid(const std::string& email)
{
	const std::regex pattern("(\\w+)(\\.|_)?(\\w*)@(\\w+)(\\.(\\w+))+");

	return std::regex_match(email, pattern);
}

int main()
{
	std::string email = "marius_b@domain.co.uk";

	std::cout << email << " : " << (IsEmailValid(email) ? "valid" : "invalid") << std::endl;

	system("pause");

	return 0;
}
