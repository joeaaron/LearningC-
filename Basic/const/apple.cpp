class Apple
{
private:
	int people[100];
	const int apple_number;
	//使用c++11标准编译
	//static const int apple_number = 10;
public:
	Apple(int i);
	void take(int num) const;
	void add(int num);
	void add(int num) const;
	int getCount() const;
};