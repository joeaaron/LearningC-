#include <iostream>
#include <tuple>
#include <string>
#include <boost/variant.hpp>

auto GetStudent(int nId)
{
	if (0 == nId)
		return std::make_tuple(3.8, 'A', "张三");
	if (1 == nId)
		return std::make_tuple(2.9, 'C', "李四");
	if (2 == nId)
		return std::make_tuple(1.7, 'D', "王五");

	return std::make_tuple(0.0, 'D', "null");
}

int main()
{
	auto student = GetStudent(0);
	std::cout << "ID : O, "
		<< "GPA: " << std::get<0>(student) << ", "
		<< "成绩: " << std::get<1>(student) << ", "
		<< "姓名: " << std::get<2>(student) << '\n';

	double dGpa;
	char cGrade;
	std::string strName;

	// 元组进行拆包
	std::tie(dGpa, cGrade, strName) = GetStudent(1);
	std::cout << "ID : 1, "
		<< "GPA: " << dGpa << ", "
		<< "成绩: " << cGrade << ", "
		<< "姓名: " << strName << '\n';

	std::tuple<std::string, double, double, int> t("123", 4.5, 6.7, 8);

}