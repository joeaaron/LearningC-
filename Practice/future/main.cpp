#include <iostream>
#include <future>
#include <thread>


int main()
{
	// ��һ������ֵΪ7��lambda���ʽ��װ��task��
	// std::packaged_task��ģ�����ΪҪ��װ����������
	std::packaged_task<int()> task([]() { return 7; });
	// ���task��future
	std::future<int> result = task.get_future();		// ��һ���߳���ִ��task
	std::thread(std::move(task)).detach();
	std::cout << "Waiting...";
	result.wait();

	// ���ִ�н��
	std::cout << "Done!" << std::endl << "Result is " << result.get() << '\n';

}