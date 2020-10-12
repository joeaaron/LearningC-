#include <iostream>
#include "Tracer.h"

bool Tracer::m_bReady = false;

Tracer::Tracer() :m_LockCount(0)
{
	m_bReady = true;		// ��ʼ����
}

Tracer::~Tracer()
{
	m_bReady = false;
	Dump();
}

// ���ڴ�й©����Ϣ�ŵ�map������
void Tracer::Add(void* p, const char* file, long line)
{
	if (m_LockCount > 0)			// ��ֹ�ݹ����
		return;
	// ���þֲ����������Թ�����Զ���������������
	Tracer::Lock lock(*this);		// ��ֹ�ڲ����ٴε���new->add->new->add...
	m_MapEntry[p] = Entry(file, line);
}

void Tracer::Remove(void* p)
{
	if (m_LockCount > 0)
		return;
	Tracer::Lock lock(*this);
	auto it = m_MapEntry.find(p);	// auto�Զ����ͣ������Զ��Ƶ����ұߵ�����
	if (it != m_MapEntry.end())
	{
		m_MapEntry.erase(it);
	}
}

// ����ڴ�й©�ĵط�
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
			int addr = reinterpret_cast<int>(it->first);	//��ָ��ת��������
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
		NewTrace.Add(p, file, line);				// �ѿ��ܷ����ڴ�й©����Ϣ��ӽ�ȥ
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
		NewTrace.Add(p, "?", 0);//�ѿ��ܷ����ڴ�й©����Ϣ��ӽ�ȥ
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
		NewTrace.Add(p, file, line);//�ѿ��ܷ����ڴ�й©����Ϣ��ӽ�ȥ
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
		NewTrace.Add(p, "?", 0);//�ѿ��ܷ����ڴ�й©����Ϣ��ӽ�ȥ
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