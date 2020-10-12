#pragma once
#include<map>

#ifndef DEBUG  //Debug�汾�Ÿ���

//����ȫ��new,delete
void* operator new(size_t size, const char* file, long line);
void operator delete(void* p, const char*, long);

void* operator new(size_t size);
void operator delete(void* p);

void* operator new[](size_t size, const char* file, long line);
void operator delete[](void* p, const char*, long);

void* operator new[](size_t size);
void operator delete[](void* p);

// �ڴ������
class Tracer
{
public:
	Tracer();
	~Tracer();
	void Add(void* p, const char* file, long line);			// ���ڴ�й©����Ϣ�ŵ�map������
	void Remove(void* p);									// �Ƴ��ڴ�й©��Ϣ
	void Dump();											// ��ӡ����ڴ�й©��Ϣ
	static bool m_bReady;									// �Ƿ�ʼ����
private:
	// Ƕ����
	class Entry
	{
	public:
		Entry(const char* file = 0, long line = 0)
			:m_File(file), m_Long(line) {}
		const char* File() const { return m_File; }			// �����ļ���
		long line() const { return m_Long; }				// ������
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

	//һ�������Lock���ǲ��ܷ���Tracer���˽�г�Ա��
	//Ϊ�˷�ֹ��Щ��������������ͨ�����룬��������Ԫ��
	friend class Lock; //��Ԫ������˷�װ��,��Ԫ�����ƻ��˷�װ��
private:
	std::map<void*, Entry> m_MapEntry;
	int m_LockCount; //������
	void lock() { ++m_LockCount; }				//����
	void unlock() { --m_LockCount; }			//����
};

extern Tracer NewTrace;

#endif