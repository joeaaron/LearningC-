# Length of Last Word

Easy

Given a string `s` consisting of some words separated by some number of spaces, return *the length of the **last** word in the string.*

A **word** is a maximal substring consisting of non-space characters only.

 

**Example 1:**

```
Input: s = "Hello World"
Output: 5
Explanation: The last word is "World" with length 5.
```

**Example 2:**

```
Input: s = "   fly me   to   the moon  "
Output: 4
Explanation: The last word is "moon" with length 4.
```

**Example 3:**

```
Input: s = "luffy is still joyboy"
Output: 6
Explanation: The last word is "joyboy" with length 6.
```

 

**Constraints:**

- `1 <= s.length <= 104`
- `s` consists of only English letters and spaces `' '`.
- There will be at least one word in `s`.

方法一：反向遍历
题目要求得到字符串中最后一个单词的长度，可以反向遍历字符串，寻找最后一个单词并计算其长度。

由于字符串中至少存在一个单词，因此字符串中一定有字母。首先找到字符串中的最后一个字母，该字母即为最后一个单词的最后一个字母。

从最后一个字母开始继续反向遍历字符串，直到遇到空格或者到达字符串的起始位置。遍历到的每个字母都是最后一个单词中的字母，因此遍历到的字母数量即为最后一个单词的长度。

