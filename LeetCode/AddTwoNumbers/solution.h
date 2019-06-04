#include <cstddef>
#include <cstdlib>

struct ListNode {
    int val;
    ListNode *next;
    ListNode(int x) : val(x), next(nullptr) {}
};

class Solution {
public:
    ListNode *addTwoNumbers(ListNode *l1, ListNode *l2) {
        ListNode dummy(0), *tail = &dummy;
        for (div_t sum{0, 0}; sum.quot || l1 || l2; tail = tail->next) {
            if (l1) { sum.quot += l1->val; l1 = l1->next; }		// 进位 + l1 
            if (l2) { sum.quot += l2->val; l2 = l2->next; }		// 进位 + l1 + l2
            sum = div(sum.quot, 10);				 // 除 10 操作，得到新的 quot 与 rem.
			tail->next = new ListNode(sum.rem);		 // rem 为节点值, quot 留作下次迭代
        }
        return dummy.next;
    }
};


