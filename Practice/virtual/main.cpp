/*
某公司有经理、销售员、小时工等多类人员。经理按周计算薪金；销售员每月底薪800元，
然后加销售提成，每销售1件产品提取销售利润的5%；小时工按小时计算薪金。每类人员都有姓名和身份证号等数据。
为简化问题，把各类人员的共有信息抽象成基类Employee，其他人员则继承该类的功能。
*/
#include <iostream>
#include <string>
using namespace std;

class Employee
{
public:
	Employee(string name, string id){ name = Name; id = Id; }
	string getName(){ return Name; }
	string getID(){ return Id; }
	virtual float getSalary(){ return 0.0; }
	virtual void print(){
		cout << "姓名：" << Name << "\t\t 编号：" << Id << endl;

	}
private:
	string Name;
	string Id;
};

class Manager : public Employee
{
public:
	Manager(string Name, string Id, int week) :Employee(Name, Id){
		weeklySalary = week * 1000;
	}
	float getSalary(){ return weeklySalary; }	//获取经理的周薪
	void print(){			//打印经理姓名、身份证、周薪
		cout << "经理：" << getName() << "\t\t 编号: " << getID()
			<< "\t\t 总工资: " << getSalary() << endl;
	}
private:
	float weeklySalary;             //周薪
};

class SaleWorker :public Employee{
public:
	SaleWorker(string name, string id, int profit, int x) :Employee(name, id){
		workerMoney = baseMoney + x*0.05*profit;
	}
	float getSalary(){
		return workerMoney;
	}
	void print(){
		cout << "销售员：" << getName() << "\t\t 编号: " << getID()
			<< "\t\t 总工资: " << getSalary() << endl;
	}
private:
	float baseMoney = 800.0;
	float workerMoney;
};

class HourWorker :public Employee{
public:
	HourWorker(string name, string id, int h) :Employee(name, id){
		TotalMoney = h*hourMoney;
	}
	float getSalary(){
		return TotalMoney;
	}
	void print(){
		cout << "小时工：" << getName() << "\t\t 编号: " << getID()
			<< "\t\t 总工资: " << getSalary() << endl;
	}
private:
	float hourMoney = 100.0;
	float TotalMoney;
};

int main(){
	cout << "请输入工作周：";
	int week;
	cin >> week;
	Manager m("小王", "11111111", week);
	m.print();
	cout << "请输入销售利润：";
	int profit;
	cin >> profit;
	cout << "请输入销售件数：";
	int x;
	cin >> x;
	SaleWorker s("小李", "222222", profit, x);
	s.print();
	cout << "请输入工作小时：";
	int hour;
	cin >> hour;
	HourWorker h("小何", "333333", hour);
	h.print();
	system("pause");
	return 0;
}