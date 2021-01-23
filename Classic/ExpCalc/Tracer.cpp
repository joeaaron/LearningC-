#include <iostream>
#include "Tracer.h"

bool Tracer::m_bReady = false;

Tracer::Tracer() :m_LockCount(0)
{
	m_bReady = true;		// 开始跟踪
}

Tracer::~Tracer()
{
	m_bReady = false;
	Dump();
}

// 把内存泄漏的信息放到map容器中
void Tracer::Add(void* p, const char* file, long line)
{
	if (m_LockCount > 0)			// 防止递归调用
		return;
	// 利用局部类对象的特性构造和自动析构来加锁解锁
	Tracer::Lock lock(*this);		// 防止内部库再次调用new->add->new->add...
	m_MapEntry[p] = Entry(file, line);
}

void Tracer::Remove(void* p)
{
	if (m_LockCount > 0)
		return;
	Tracer::Lock lock(*this);
	auto it = m_MapEntry.find(p);	// auto自动类型，可以自动推导出右边的类型
	if (it != m_MapEntry.end())
	{
		m_MapEntry.erase(it);
	}
}

// 输出内存泄漏的地方
void Tracer::Dump()
{
	if (m_MapEntry.size() > 0)
	{
		std::cout << "*** Memory leak(s):" << std::endl;
		auto it = m_MapEntry.begin();
		for (; it != m_MapEntry.end(); it++)
		{
			const char* file = it->second.File();
			long line = it->second.line();
			int addr = reinterpret_cast<int>(it->first);	//把指针转换成整型
			std::cout << "0x" << std::hex << addr << ": "
				<< file << ", line" << std::dec << line << std::endl;
		}
		std::cout << std::endl;
	}
}

void* operator new(size_t size, const char* file, long line)
{
	void* p = malloc(size);
	if (Tracer::m_bReady)
	{
		NewTrace.Add(p, file, line);				// 把可能发生内存泄漏的信息添加进去
	}
	return p;
}

void operator delete(void* p, const char*, long)
{
	if (Tracer::m_bReady)
	{
		NewTrace.Remove(p);
	}
	free(p);
}

void* operator new(size_t size)
{
	void* p = malloc(size);
	if (Tracer::m_bReady)
	{
		NewTrace.Add(p, "?", 0);//把可能发生内存泄漏的信息添加进去
	}
	return p;
}

void operator delete(void* p)
{
	if (Tracer::m_bReady)
	{
		NewTrace.Remove(p);
	}
	free(p);
}


void* operator new[](size_t size, const char* file, long line)
{
	void* p = malloc(size);
	if (Tracer::m_bReady)
	{
		NewTrace.Add(p, file, line);//把可能发生内存泄漏的信息添加进去
	}
	return p;
}
void operator delete[](void* p, const char*, long)
{
	if (Tracer::m_bReady)
	{
		NewTrace.Remove(p);
	}
	free(p);
}

void* operator new[](size_t size)
{
	void* p = malloc(size);
	if (Tracer::m_bReady)
	{
		NewTrace.Add(p, "?", 0);//把可能发生内存泄漏的信息添加进去
	}
	return p;
}
void operator delete[](void* p)
{
	if (Tracer::m_bReady)
	{
		NewTrace.Remove(p);
	}
	free(p);
}

Tracer NewTrace;