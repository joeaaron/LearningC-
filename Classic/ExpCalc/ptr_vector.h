#pragma once

template <typename T>
class ptr_vector : public std::vector<T*>
{
public:
	void clear()
	{
		std::vector<T*>::iterator it = begin();
		for (; it != end(); it++)
			delete *it;				// 删除指针所指向的内容
		std::vector<T*>::clear();	// 删除指针变量本身
	}
	~ptr_vector()
	{
		clear();
	}
	void push_back(T* const& val)
	{
		std::auto_ptr<T> ptr(val);			// 转移所有权到智能指针中
		std::vector<T*>::push_back(val)		// 防止内部分配内存失败,导致指针不能释放,所以要使用智能指针管理
		ptr.release();						// 释放所有权
	}
	void push_back(std::auto_ptr<T>& val)
	{
		std::vector<T*>::push_back(val.get());  // 将原生指针传进去
		val.release();							// 插入成功后释放所有权
	}
};