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
	//无参构造的初始化
	Vector() :p(NULL), capacity(0), size(0)
	{
		//this->capacity = 20;
		//this->size = 0;
		//T *p = new T[capacity];
	}
	//有参构造的初始化方式
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
	//析构函数，释放掉唯一的指针
	~Vector()
	{
		if (p != NULL)
		{
			delete[] p;
		}
	}
	//拷贝构造函数
	Vector(const Vector& v)
	{
		//令人发指，如果直接调用拷贝构造，虽然没给成员变量p赋值，但这个指针不是空的
		//等号赋值那里可以加上判断是否为空
		this->capacity = v.capacity;
		this->size = v.size;
		this->p = new T[this->capacity];
		memcpy(this->p, v.p, this->size*sizeof(T));
	}

	//打印容器内容
	void print()
	{
		for (int i = 0; i < this->size; i++)
		{
			cout << this->p[i] << '\t';
		}
		cout << endl;
	}
	//插入，要判断溢出没
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
			//如果满了，每次容量拓展到2倍
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
	//删除最后一个元素，无返回值
	void pop_back()
	{
		if (this->size > 1)
		{
			this->p[this->size - 1] = 0;
			this->size--;
		}
	}
	//插入
	void insert(int pos, T data)
	{
		if (pos >= 0 && pos <= this->size)
		{

			if (this->size == this->capacity)
			{
				//如果满了，每次容量拓展到2倍
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
	//清除，很佛系，假装清除了
	void clear()
	{
		this->size = 0;
	}
	//重载[]运算符，可以用[]修改函数
	T& operator[](int index)
	{
		if (index > 0 && index < this->size)
		{
			return this->p[index];
		}
	}
	//重载赋值=，其实和拷贝构造函数一个样
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