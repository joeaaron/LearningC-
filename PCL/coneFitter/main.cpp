#include <iostream>
#include <fstream>
#include "ConeFitter.h"

using namespace std;

std::vector<Point> ReadPointsFromFile(const std::string& filename)
{
	std::vector<Point> points;
	std::ifstream file(filename);

	if (!file.is_open()) {
		std::cerr << "Could not open the file!" << std::endl;
		return points;
	}

	std::string line;
	while (std::getline(file, line))
	{
		std::replace(line.begin(), line.end(), ',', ' ');
		std::istringstream iss(line);
		double x, y, z;
		if (!(iss >> x >> y >> z))
		{
			std::cerr << "Error parsing line: " << line << std::endl;
			break;
		}
		points.emplace_back(Point(x, y, z));
	}

	file.close();
	return points;
}

int main()
{
	Gauss::ConeFitter coneFitter;
	double err;
	vector<Point> points = ReadPointsFromFile("ConePart1.txt");
	Fitting::Cone fitResult;
	err = coneFitter.Fitting(points, &fitResult);

	return 0;
}

