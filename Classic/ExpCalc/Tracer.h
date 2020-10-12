#pragma once
#include<map>

#ifndef DEBUG  //Debug版本才跟踪

//重载全局new,delete
void* operator new(size_t size, const char* file, long line);
void operator delete(void* p, const char*, long);

void* operator new(size_t size);
void operator delete(void* p);

void* operator new[](size_t size, const char* file, long line);
void operator delete[](void* p, const char*, long);

void* operator new[](size_t size);
void operator delete[](void* p);

// 内存跟踪类
class Tracer
{
public:
	Tracer();
	~Tracer();
	void Add(void* p, const char* file, long line);			// 把内存泄漏的信息放到map容器中
	void Remove(void* p);									// 移除内存泄漏信息
	void Dump();											// 打印输出内存泄漏信息
	static bool m_bReady;									// 是否开始跟踪
private:
	// 嵌套类
	class Entry
	{
	public:
		Entry(const char* file = 0, long line = 0)
			:m_File(file), m_Long(line) {}
		const char* File() const { return m_File; }			// 返回文件名
		long line() const { return m_Long; }				// 返回行
	private:
		const char* m_File;
		long m_Long;
	};

	class Lock
	{
	public:
		Lock(Tracer& tracer) : m_Tracer(tracer)
		{
			m_Tracer.lock();
		}
		~Lock()
		{
			m_Tracer.unlock();
		}
	private:
		Tracer& m_Tracer;

	};

	//一般情况下Lock类是不能访问Tracer类的私有成员的
	//为了防止有些编译器不能正常通过编译，最后加上友元类
	friend class Lock; //友元类提高了封装性,友元函数破坏了封装性
private:
	std::map<void*, Entry> m_MapEntry;
	int m_LockCount; //计数器
	void lock() { ++m_LockCount; }				//加锁
	void unlock() { --m_LockCount; }			//解锁
};

extern Tracer NewTrace;

#endif