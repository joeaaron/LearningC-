#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

class logfile
{
	boost::mutex m_mutex;
	std::ofstream f;
public:
	logfile()   //constructor
	{
		f.open("boostlog.txt");
	}

	void shared_print(std::string id, int value)
	{
		boost::lock_guard<boost::mutex> locker(m_mutex); //it uses RAII 
		f << "from" << id << ":" << value << endl;
	}

};
void function_1(logfile & log)
{
	for (int i = 0; i > -20; i--)
		log.shared_print(string("from t1"), i);
}
int main()
{
	logfile log; // pass the log to thread t1 by the reference.
	boost::thread t1(function_1, boost::ref(log));
	for (int i = 0; i < 20; i++)
		log.shared_print(string("from main:"), i);
	t1.join();

	system("pause");
	return 0;
}