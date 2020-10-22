/*
ĳ��˾�о���������Ա��Сʱ���ȶ�����Ա���������ܼ���н������Աÿ�µ�н800Ԫ��
Ȼ���������ɣ�ÿ����1����Ʒ��ȡ���������5%��Сʱ����Сʱ����н��ÿ����Ա��������������֤�ŵ����ݡ�
Ϊ�����⣬�Ѹ�����Ա�Ĺ�����Ϣ����ɻ���Employee��������Ա��̳и���Ĺ��ܡ�
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
		cout << "������" << Name << "\t\t ��ţ�" << Id << endl;

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
	float getSalary(){ return weeklySalary; }	//��ȡ��������н
	void print(){			//��ӡ��������������֤����н
		cout << "������" << getName() << "\t\t ���: " << getID()
			<< "\t\t �ܹ���: " << getSalary() << endl;
	}
private:
	float weeklySalary;             //��н
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
		cout << "����Ա��" << getName() << "\t\t ���: " << getID()
			<< "\t\t �ܹ���: " << getSalary() << endl;
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
		cout << "Сʱ����" << getName() << "\t\t ���: " << getID()
			<< "\t\t �ܹ���: " << getSalary() << endl;
	}
private:
	float hourMoney = 100.0;
	float TotalMoney;
};

int main(){
	cout << "�����빤���ܣ�";
	int week;
	cin >> week;
	Manager m("С��", "11111111", week);
	m.print();
	cout << "��������������";
	int profit;
	cin >> profit;
	cout << "���������ۼ�����";
	int x;
	cin >> x;
	SaleWorker s("С��", "222222", profit, x);
	s.print();
	cout << "�����빤��Сʱ��";
	int hour;
	cin >> hour;
	HourWorker h("С��", "333333", hour);
	h.print();
	system("pause");
	return 0;
}