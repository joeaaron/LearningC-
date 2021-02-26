# Add Two Numbers

You are given two **non-empty** linked lists representing two non-negative integers. The digits are stored in **reverse order** and each of their nodes contain a single digit. Add the two numbers and return it as a linked list.

You may assume the two numbers do not contain any leading zero, except the number 0 itself.

**Example:**

```
Input: (2 -> 4 -> 3) + (5 -> 6 -> 4)
Output: 7 -> 0 -> 8
Explanation: 342 + 465 = 807.
```

**Code:**

```
class Solution {
public:
	ListNode *addTwoNumbers(ListNode *l1, ListNode *l2) {
		ListNode dummy(0), *tail = &dummy;
		for (div_t sum{ 0, 0 }; sum.quot || l1 || l2; tail = tail->next) {
			if (l1) { sum.quot += l1->val; l1 = l1->next; }		// 进位 + l1 
			if (l2) { sum.quot += l2->val; l2 = l2->next; }		// 进位 + l1 + l2
			sum = div(sum.quot, 10);							// 除 10 操作，得到新的 quot 与 rem.
			tail->next = new ListNode(sum.rem);					// rem 为节点值, quot 留作下次迭代
		}
		return dummy.next;
	}
};
```

这道题难点在于**进位**；其次，这种关系到两个链表的问题，dummy 节点应该是必不可少的了。

- div_t: Structure to represent the result value of an integral division performed by function [div](http://www.cplusplus.com/div).

  It has two `int` members: quot and rem, defined in either order. A possible definition could be:  

  ```
  typedef struct {
    int quot;
    int rem;
  } div_t;
  ```