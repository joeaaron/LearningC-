# Longest Substring Without Repeating Characters

Given a string, find the length of the **longest substring** without repeating characters.

**Example 1:**

```
Input: "abcabcbb"
Output: 3 
Explanation: The answer is "abc", with the length of 3. 
```

**Example 2:**

```
Input: "bbbbb"
Output: 1
Explanation: The answer is "b", with the length of 1.
```

**Example 3:**

```
Input: "pwwkew"
Output: 3
Explanation: The answer is "wke", with the length of 3. 
             Note that the answer must be a substring, "pwke" is a subsequence and not a substring.
```

 **Code:**

```
class Solution {
public:
	int lengthOfLonestSubstring(string s){
		size_t ret = 0, start = 0;
		unordered_map<char, size_t> trace;        //来边记录边比较(max)
		for (size_t i = 0; i < s.size(); ++i){
			auto found = trace.find(s[i]);
			if (found != trace.end() && found->second >= start){
				ret = max(ret, i - start);
				start = found->second + 1;
			}
			trace[s[i]] = i;
		}
		return max(ret, s.size() - start);
	}
};
```

首先，求的只是长度，那么一定有一个 trace 来边记录边比较(max)。 其次，没有重复字符几乎是唯一条件，那么检查重复显然用 k-v。 最后，要考虑一次迭代过程中，如何度量这个长度。

------

设 substr 的起点为 start(s), 终点为 last(l). 每一次迭代，记录一张索引表。

```
abcabcbb
^  ^
s  l
```

| char | pos  |
| ---- | ---- |
| a    | 0    |
| b    | 1    |
| c    | 2    |

上图所示，last 指向 `a`, 查询当前表可知，`a` 的位置记录在案，且 `pos >= start`. 故此刻诞生一个 substr. 长度为 `last - start`. s 更新位置为 `pos + 1`.