Given an array of integers, return **indices** of the two numbers such that they add up to a specific target.

You may assume that each input would have **exactly** one solution, and you may not use the *same* element twice.

**Example:**

```
Given nums = [2, 7, 11, 15], target = 9,

Because nums[0] + nums[1] = 2 + 7 = 9,
return [0, 1].
```

 **Code:**

```
class Solution {
public:
    vector<int> twoSum(vector<int>& nums, int target) {
        std::unordered_map<int, int> record;
        for (int i = 0; i != nums.size(); ++i) {
            auto found = record.find(nums[i]);
            if (found != record.end())
                return {found->second, i};
            record.emplace(target - nums[i], i);
        }
        return {-1, -1};
    }
};
```

这道题有点像找配对。找到 `a`，找到 `b`，让 `a + b = target`。遍历的过程中，记录什么，成了思考的关键。

既然是配对，那么 kv 结构是非常合适的，于是上了 Hash 表。让 key 就是要找的 `b`，让 value 记录 `a` 的 index，也就是结果需要的索引。

- [unordered_map::find](http://www.cplusplus.com/reference/unordered_map/unordered_map/find/)   

  Searches the container for an element with *k* as key and returns an iterator to it if found, otherwise it returns an iterator to [unordered_map::end](http://www.cplusplus.com/unordered_map::end) (the element past the end of the container).

- unordered_map::emplace

  Inserts a new element in the [unordered_map](http://www.cplusplus.com/unordered_map) if its key is unique. This new element is constructed in place using *args* as the arguments for the element's constructor.

  The insertion only takes place if no element in the container has a key equivalent to the one being emplaced (keys in an [unordered_map](http://www.cplusplus.com/unordered_map) are unique).

  If inserted, this effectively increases the container [size](http://www.cplusplus.com/unordered_map::size) by one.

  A similar member function exists, [insert](http://www.cplusplus.com/unordered_map::insert), which either copies or moves existing objects into the container.  

------

**推荐阅读：**

【1】[c++中map与unordered_map的区别](https://blog.csdn.net/batuwuhanpei/article/details/50727227)

【2】[What is the difference between unordered_map :: emplace and unordered_map :: insert in C++?](https://stackoverflow.com/questions/26446352/what-is-the-difference-between-unordered-map-emplace-and-unordered-map-ins)