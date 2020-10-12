#pragma once

template <typename T>
class ptr_vector : public std::vector<T*>
{
public:
	void clear()
	{
		std::vector<T*>::iterator it = begin();
		for (; it != end(); it++)
			delete *it;				// ɾ��ָ����ָ�������
		std::vector<T*>::clear();	// ɾ��ָ���������
	}
	~ptr_vector()
	{
		clear();
	}
	void push_back(T* const& val)
	{
		std::auto_ptr<T> ptr(val);			// ת������Ȩ������ָ����
		std::vector<T*>::push_back(val)		// ��ֹ�ڲ������ڴ�ʧ��,����ָ�벻���ͷ�,����Ҫʹ������ָ�����
		ptr.release();						// �ͷ�����Ȩ
	}
	void push_back(std::auto_ptr<T>& val)
	{
		std::vector<T*>::push_back(val.get());  // ��ԭ��ָ�봫��ȥ
		val.release();							// ����ɹ����ͷ�����Ȩ
	}
};