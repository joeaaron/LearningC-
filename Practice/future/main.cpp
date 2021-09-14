#include <iostream>
#include <future>
#include <thread>


int main()
{
	// 将一个返回值为7的lambda表达式封装到task中
	// std::packaged_task的模板参数为要封装函数的类型
	std::packaged_task<int()> task([]() { return 7; });
	// 获得task的future
	std::future<int> result = task.get_future();		// 在一个线程中执行task
	std::thread(std::move(task)).detach();
	std::cout << "Waiting...";
	result.wait();

	// 输出执行结果
	std::cout << "Done!" << std::endl << "Result is " << result.get() << '\n';

}