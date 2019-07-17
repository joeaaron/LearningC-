# Median of Two Sorted Arrays

There are two sorted arrays **nums1** and **nums2** of size m and n respectively.

Find the median of the two sorted arrays. The overall run time complexity should be O(log (m+n)).

You may assume **nums1** and **nums2** cannot be both empty.

**Example 1:**

```
nums1 = [1, 3]
nums2 = [2]

The median is 2.0
```

**Example 2:**

```
nums1 = [1, 2]
nums2 = [3, 4]

The median is (2 + 3)/2 = 2.5
```

 **Code:**

```
class Solution {
public:
	double findMedianSortedArrays(vector<int>& nums1, vector<int>& nums2) {
		int N1 = nums1.size();
		int N2 = nums2.size();
		if (N1 < N2) return findMedianSortedArrays(nums2, nums1);	// Make sure A2 is the shorter one.

		int lo = 0, hi = N2 * 2;
		while (lo <= hi) {
			int mid2 = (lo + hi) / 2;   // Try Cut 2 
			int mid1 = N1 + N2 - mid2;  // Calculate Cut 1 accordingly

			double L1 = (mid1 == 0) ? INT_MIN : nums1[(mid1 - 1) / 2];	// Get L1, R1, L2, R2 respectively
			double L2 = (mid2 == 0) ? INT_MIN : nums2[(mid2 - 1) / 2];
			double R1 = (mid1 == N1 * 2) ? INT_MAX : nums1[(mid1) / 2];
			double R2 = (mid2 == N2 * 2) ? INT_MAX : nums2[(mid2) / 2];

			if (L1 > R2) lo = mid2 + 1;		// A1's lower half is too big; need to move C1 left (C2 right)
			else if (L2 > R1) hi = mid2 - 1;	// A2's lower half too big; need to move C2 left.
			else return (max(L1, L2) + min(R1, R2)) / 2;	// Otherwise, that's the right cut.
		}
		return -1;
	}
};

```

**解题思路**

要求在O(log (m+n))时间内找到中位数，所以像那些合并之后再二分查找、或者一边比较一边合并到总量一半的方法肯定是不行的。

我们可以将原问题转变成一个**寻找第k小数**的问题（假设两个原序列升序排列），这样中位数实际上是第(m+n)/2小的数。所以只要解决了第k小数的问题，原问题也得以解决。

首先假设数组A和B的元素个数都大于k/2，我们比较A[k/2-1]和B[k/2-1]两个元素，这两个元素分别表示A的第k/2小的元素和B的第k/2小的元素。这两个元素比较共有三种情况：>、<和=。如果A[k/2-1] < B[k/2-1]，这表示A[0]到A[k/2-1]的元素都在A和B合并之后的前k小的元素中。换句话说，A[k/2-1]不可能大于两数组合并之后的第k小值，所以我们可以将其抛弃。

当A[k/2-1]=B[k/2-1]时，我们已经找到了第k小的数，也即这个相等的元素，我们将其记为m。由于在A和B中分别有k/2-1个元素小于m，所以m即是第k小的数。(这里可能有人会有疑问，如果k为奇数，则m不是中位数。这里是进行了理想化考虑，在实际代码中略有不同，是先求k/2，然后利用k-k/2获得另一个数。)

通过上面的分析，我们即可以采用递归的方式实现寻找第k小的数。此外我们还需要考虑几个边界条件：

- 如果A或者B为空，则直接返回B[k-1]或者A[k-1]； 
- 如果k为1，我们只需要返回A[0]和B[0]中的较小值； 
- 如果A[k/2-1]=B[k/2-1]，返回其中一个；