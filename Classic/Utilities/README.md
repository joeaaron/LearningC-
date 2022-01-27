就学习C++ 语言本身而言，我认为有几个练习非常值得一做。这不是“重复发明轮子”，而是必要的编程练习，帮助你熟悉掌握这门语言。

- 一是写一个复数类或者大整数类，实现基本的加减乘运算，**熟悉封装与数据抽象**。

- 二是写一个字符串类，**熟悉内存管理与拷贝控制**。

- 三是写一个简化的vector<T> 类模板，**熟悉基本的模板编程**，你的这个vector 应该能放入int 和std::string 等元素类型。

- 四是写一个表达式计算器，实现一个节点类的继承体系（右图），体会**面向对象编程**。

前三个练习是写独立的值语义的类，第四个练习是对象语义，同时要考虑类与类之间的关系。表达式计算器能把四则运算式3+2*4 解析为左图的表达式树，对根节点调用calculate() 虚函数就能算出表达式的值。做完之后还可以再扩充功能，比如支持三角函数和变量。

![image-20200927162303267](C:\Users\P03918\AppData\Roaming\Typora\typora-user-images\image-20200927162303267.png)