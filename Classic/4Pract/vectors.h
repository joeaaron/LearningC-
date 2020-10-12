#pragma once

#include <iostream>
#include <cstring>
using namespace std;

template <typename T>
class Vector
{
public:
	T *p;
	int capacity;
	int size;
public:
	//�޲ι���ĳ�ʼ��
	Vector() :p(NULL), capacity(0), size(0)
	{
		//this->capacity = 20;
		//this->size = 0;
		//T *p = new T[capacity];
	}
	//�вι���ĳ�ʼ����ʽ
	Vector(int size, T data)
	{
		this->capacity = 20 + size;
		this->size = size;
		this->p = new T[capacity];
		for (int i = 0; i < this->size; i++)
		{
			this->p[i] = data;
		}
	}
	//�����������ͷŵ�Ψһ��ָ��
	~Vector()
	{
		if (p != NULL)
		{
			delete[] p;
		}
	}
	//�������캯��
	Vector(const Vector& v)
	{
		//���˷�ָ�����ֱ�ӵ��ÿ������죬��Ȼû����Ա����p��ֵ�������ָ�벻�ǿյ�
		//�ȺŸ�ֵ������Լ����ж��Ƿ�Ϊ��
		this->capacity = v.capacity;
		this->size = v.size;
		this->p = new T[this->capacity];
		memcpy(this->p, v.p, this->size*sizeof(T));
	}

	//��ӡ��������
	void print()
	{
		for (int i = 0; i < this->size; i++)
		{
			cout << this->p[i] << '\t';
		}
		cout << endl;
	}
	//���룬Ҫ�ж����û
	void push_back(T data)
	{
		if (this->p == NULL)
		{
			this->capacity = 20;
			this->size = 0;
			T *p = new T[capacity];
		}
		if (this->size == this->capacity)
		{
			//������ˣ�ÿ��������չ��2��
			T *new_p = new T[this->capacity * 2];
			memcpy(new_p, p, this->size*sizeof(T));
			this->capacity *= 2;
			delete[] p;
			p = new_p;

		}
		this->print();
		this->p[this->size] = data;
		this->size++;
	}
	//ɾ�����һ��Ԫ�أ��޷���ֵ
	void pop_back()
	{
		if (this->size > 1)
		{
			this->p[this->size - 1] = 0;
			this->size--;
		}
	}
	//����
	void insert(int pos, T data)
	{
		if (pos >= 0 && pos <= this->size)
		{

			if (this->size == this->capacity)
			{
				//������ˣ�ÿ��������չ��2��
				T *new_p = new T[this->capacity * 2];
				memcpy(new_p, p, this->size*sizeof(T));
				this->capacity *= 2;
				delete[] p;
				p = new_p;
			}
			for (int i = this->size; i > pos; i--)
			{
				this->p[i] = this->p[i - 1];
			}
			this->p[pos] = data;
			this->size++;

		}
	}
	//������ܷ�ϵ����װ�����
	void clear()
	{
		this->size = 0;
	}
	//����[]�������������[]�޸ĺ���
	T& operator[](int index)
	{
		if (index > 0 && index < this->size)
		{
			return this->p[index];
		}
	}
	//���ظ�ֵ=����ʵ�Ϳ������캯��һ����
	void operator=(const Vector& v)
	{
		if (this->p != NULL)
		{
			delete[] this->p;
			this->capacity = 0;
			this->size = 0;
			this->p = NULL;
		}
		this->capacity = v.capacity;
		this->size = v.size;
		this->p = new T[this->capacity];
		memcpy(this->p, v.p, this->size*sizeof(T));
	}
};