#include <iostream>
#include <tuple>
#include <string>
#include <boost/variant.hpp>

auto GetStudent(int nId)
{
	if (0 == nId)
		return std::make_tuple(3.8, 'A', "����");
	if (1 == nId)
		return std::make_tuple(2.9, 'C', "����");
	if (2 == nId)
		return std::make_tuple(1.7, 'D', "����");

	return std::make_tuple(0.0, 'D', "null");
}

int main()
{
	auto student = GetStudent(0);
	std::cout << "ID : O, "
		<< "GPA: " << std::get<0>(student) << ", "
		<< "�ɼ�: " << std::get<1>(student) << ", "
		<< "����: " << std::get<2>(student) << '\n';

	double dGpa;
	char cGrade;
	std::string strName;

	// Ԫ����в��
	std::tie(dGpa, cGrade, strName) = GetStudent(1);
	std::cout << "ID : 1, "
		<< "GPA: " << dGpa << ", "
		<< "�ɼ�: " << cGrade << ", "
		<< "����: " << strName << '\n';

	std::tuple<std::string, double, double, int> t("123", 4.5, 6.7, 8);

}