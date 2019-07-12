#include <iostream>
#include <cstddef>
#include <cstdlib>

using namespace std;

struct ListNode {
	int val;
	ListNode *next;
	ListNode(int x) : val(x), next(nullptr) {}
};

class Solution {
public:
	ListNode *addTwoNumbers(ListNode *l1, ListNode *l2) {
		ListNode dummy(0), *tail = &dummy;
		for (div_t sum{ 0, 0 }; sum.quot || l1 || l2; tail = tail->next) {
			if (l1) { sum.quot += l1->val; l1 = l1->next; }		// 进位 + l1 
			if (l2) { sum.quot += l2->val; l2 = l2->next; }		// 进位 + l1 + l2
			sum = div(sum.quot, 10);				 // 除 10 操作，得到新的 quot 与 rem.
			tail->next = new ListNode(sum.rem);		 // rem 为节点值, quot 留作下次迭代
		}
		return dummy.next;
	}
};

ListNode *create_linkedlist(std::initializer_list<int> lst)
{
    auto iter = lst.begin();
    ListNode *head = lst.size() ? new ListNode(*iter++) : NULL;
    for (ListNode *cur=head; iter != lst.end(); cur=cur->next)
        cur->next = new ListNode(*iter++);
    return head;
}

int main()
{
    Solution s;
    ListNode *l1 = create_linkedlist({2,4,3});
    ListNode *l2 = create_linkedlist({5,6,4});
    ListNode *ret = s.addTwoNumbers(l1, l2);
    for (ListNode *cur = ret; cur; cur = cur->next)
        cout << cur->val << "->";
    cout << "\b\b  " << endl;

    return 0;
}

