# More Effective C++

> **35 个改善编程与设计的有效方法**

## 基础议题（Basics）

### 条款1：仔细区别 pointers 和 references

Pointers 与 references 的区别：

· pointers可以为null，即不指向任何对象，references必须总代表某个对象

· references一定代表某个对象，因此要求references必须要有初值，使用references可能会比pointers更富效率，因为使用references之前不需要测试其有效性，使用pointers可能检查其是否为null

· pointers可以被重新赋值，指向另一个对象，references却总指向（代表）它最初获得的那个对象

· 某些操作符必须返回某种“能够被当作assignment赋值对象”的东西时，例如operator[ ]

结论：

​    当你知道你需要指向某个东西，而且绝不会改变其他东西，或者当你实现一个操作符而其语法无法由pointers达成，你就应该选择references。任何其他时候，请采用pointers。

### 条款2：最好使用C++转型操作符

C++转型操作符（cast operators）：

**static_cast****，const_cast****，dynamic_cast****，reinterpret_cast**

static_cast基本上拥有与C旧式转型相同的威力与意义，以及相同的限制。

int first, second;

double result = static_cast<double>(first) / second;

const_cast移除表达式的常量性，专司其职，其他操作符不可以完成此任务。

const int a;

int b = const_cast<int>(a);

dynamic_cast用来执行继承体系中“安全的向下转型或跨系转型动作”，可将指向父类的指针类型转为指向其子类的指针类型。

parent *pw;        (class son : public parent)

son *sw = dynamic_cast<son*>(pw);

reinterpret_cast最常用于转换“函数指针”类型。但应尽量避免将函数指针转型。

### 条款3：绝对不要以多态（polymorphically）方式处理数组

利用多态处理数组会出现问题，原因是数组中的对象大小可能不一致。例如，类型为父类的数组与类型为子类的数组每个元素大小不一样，array和array+i之间的距离一定为i*sizeof(type)，类型不同导致子类与父类数组的距离不同，

### 条款4：非必要不提供default constructor

尽管不提供默认构造函数对于构造数组会造成一定麻烦，但对于如果仅仅为了方便构建数组而提供一个默认构造函数，在使用某个对象时就需要检查相关的数据成员是否被合理赋值，"调用者便必须为测试行为付出时间代价“，而且提供一个无意义的默认构造函数也会影响类的效率(毕竟默认构造函数的出现只是为了方便构建数组，其实什么也不做)，所以如果一个如果default constructor无法提供所有数据成员都被正确初始化，那么不对其进行定义，尽管可能对类的使用带来限制(如上文所提到的数组的构建),但也保证了使用对象时其必要的数据成员已被合理初始化。

## 操作符（Operator）

### 条款5：对定制的“类型转换函数”保持警觉

为什么最好不要提供任何类型转换函数，根本问题在于，在你从未打算也未预期的情况下，此类函数可能会被调用，而其结果可能是不正确、不直观的程序行为，很难调试。解决方法，就是用函数取代类型转换操作符。

### 条款6：区别increment/decrement操作符的前置（prefix）和后置（postfix）形式

C++中允许++ 和-- 操作符的前置和后置两种形式具有重载的能力。而重载是以参数类型来区分的，然而不论是++ 还是 -- 的前置或后置均没有参数，为了区分这两种不同的操作，只好让后置式有一个int自变量，并且在它调用的时候，编译器默认给该int指定一个0值。

char& operator++ ();        //前置式++，返回类型：引用

   const char operator++ (int);     //后置式++，返回类型：const临时对象

前置式返回调用它的对象的引用：因为直接对原对象本身进行了累加并返回自身。后置式返回const 临时对象：首先返回值必须是一个对象，这是显然的，因为要返回累加前的对象；其次为什么是const对象呢？如果不是const对象会出现什么情况：

i++++      //error! （但++++i是合法的)

如上面的代码,第二次operator++ 改变的对象是第一个operator++ 返回的对象，而不是原对象。即经过i++++之后，i的值也只是加了一次而已。这违反了我们的直觉，也违背了我们的意图，因此应该被禁止！

前置式和后置式，它们除了返回值不同，完成的任务是相同的：将某值累加！如果只需进行累加，使用前置式的效率要比后置式的高，原因有两点：

·后置式++调用了前置式++的操作；

·后置式要生成一个临时对象存储原值，这中间有拷贝构造和析构的代价，而前置式却没有。

因此，应该尽可能使用前置式操作！

### 条款7：千万不要重载&&，| | 和, 操作符

对于逻辑表达式具有“逻辑短路”的性质，即一旦确定了真假值，表达式中即使有尚未检查的部分，都将返回。

当重载&&和| |时，也希望具有上面的性质。但事与愿违，多数情况是无法达到这种要求。

当函数调用动作被执行，所有参数值都必须被评估；函数调用动作中各参数的评估顺序不确定。

if (expression1 && expresssion2)  ...

上面两种形式，都会将计算expression1和expression2的值，而且计算的顺序也不确定，这样就违背了之前谈论的“逻辑短路”现象。

故，一旦进行重载，&&和| |的左右操作数就是两个无分前后的参数而已，也就是说短路求值的特性没有了。当函数被调用时，所有参数都被求值并传入，而C++没有规定个参数的求值顺序，之前的代码就不能用了。

逗号（，）操作符的含义，是从左到右依次计算每个表达式的值，最后返回的是最后一个表达式的值，如

expression1, expression2, ..., expressionN;

将依次计算expression1，expression2，...的值，最后返回值为expressionN的值。重载逗号（，）操作符，如果是以global-function形式给出，我们无法确保参数的计算顺序是从左到右的（因为两个表达式都被当做函数调用时的自变量，传递给该操作符函数，而你无法控制一个函数的自变量的评估顺序）；如果以member-function形式给出，仍不能保证逗号操作符的左操作数先被计算（因为编译器不强迫做这样的事情）。因此，不能确保完成逗号操作符所期望的功能。

### 条款8：了解各种不同意义的new和delete

\1. new operator：new操作符，用于动态分配内存并进行初始化;

new operator 总是做这两件事一是分配足够的内存，用来放置某类型对象，二是调用构造函数为对象设定初值。无论如何你不能改变其行为。无法重载。

\2. operator new 执行必要的内存分配动作,返回值是一个指针，指向一块原始的、未设初值的内存。你可以重写或者重载这个函数,可以加额外的参数，但是第一参数类型必须是size_t。

operator new 和 malloc一样，唯一任务就是分配内存。new operator肯定会调用operator new。

\3. placement new是在已经分配好的原始内存，构建对象。这是特殊版本的operator new。

\4. delete operator<--->operator delete 就好比new operator<--->operator new，如果只打算处理原始的、未设初始值的内存，应该用operator new取得内存，并用operator delere归还给系统。

总结：

1) placement new(定位new)是new operator的另一种使用方法，用于在已分配好的内存上构造对象，它与普通的new表达式调用的是不同的operator new。

2) operator new 只分配内存而不构造对象，对象的构造，析构以及内存的释放由自己负责，提升了效率也加重了负担。

3) operator new 的使用可以与operator delete的使用相配合，new operator的使用与delete operator的使用相配合，但使用operator new分配的单个的内存也可以与delete operator配合使用；operator new[]的使用绝对不可以与delete []相配合，也就是说delete []只能释放经由new operator得到的连续内存，否则会发生未定义行为！

4) operator new[]直接调用与经由new operator调用效果是不同的，直接调用operator new[]相当于调用operator new，而调用new operator构造数组编译器还做了其他事情以便于与delete []的使用相配合。

## 异常（Exception）

### 条款9：利用destructors避免泄露资源

\1. 栈展开：

  函数抛出异常的时候，将暂停当前函数的执行，开始查找匹配的catch子句。首先检查throw本身是否在try块内部，如果是，检查与该try块相关的catch语句，看是否其中之一与被抛出的对象相匹配。如果找到匹配的catch，就处理异常;如果找不到且该try语句嵌套在其他try块中，则继续检查与外层try匹配的catch子句。若还是找不到匹配的catch，就退出当前函数(释放当前函数的内存并撤销局部对象)，并继续在调用当前函数的外层函数中查找中查找。

  若对抛出异常的函数的调用语句位于一个try语句块内，则检查与该try块关联的catch子句。若找到匹配的catch，就使用该catch处理异常。否则，若该try语句嵌套在其他try块中，则继续检查与外层try匹配的catch子句。若还是找不到匹配的catch，就退出当前主调函数，继续在调用了刚刚推出的这个函数的其他函数中继续寻找。以此类推......

\2. 函数执行的过程中一旦抛出异常，就停止接下来语句的执行，跳出try块(try块之内throw之后的语句不再执行)并开始寻找匹配的catch语句，跳出try块的过程中，会适当的撤销已经被创建的局部对象，运行局部对象的析构函数并释放内存。

\3. 如果在throw之前恰好在堆中申请了内存，而释放内存的语句又恰好在throw语句之后的话，那么一旦抛出异常，该语句将不会执行造成内存泄露问题。

\4. 解决办法是将指针类型封装在一个类中，并在该类的析构函数中释放内存。这样即使抛出异常，该类的析构函数也会运行，内存也可以被适当的释放。C++ 标准库提供了一个名为auto_ptr的类模板，用来完成这种功能。

### 条款10：在constructors内阻止资源泄露（resource leak）

\1. “C++ 只会析构已完成的对象”，“面对未完成的对象，C++ 拒绝调用其析构函数”，因为对于一个尚未构造完成的对象，构造函数不知道对象已经被构造到何种程度，也就无法析构。当然，并非不能采取某种机制使对象的数据成员附带某种指示，“指示constructor进行到何种程度，那么destructor就可以检查这些数据并(或许能够)理解应该如何应对。但这种机制无疑会降低constructor的效率，，处于效率与程序行为的取舍，C++ 并没有使用这种机制。所以说，”C++ 不自动清理那些’构造期间跑出exception‘的对象“。

\2. 当在构造函数中抛出异常的时候，由于对象尚未构造完全，因此并不会调用其析构函数，问题是此时如果对象已经被部分构造，那么我们应当保证被部分构造的内容适当地析构。一个更好的办法是，接受条款9的忠告，使用标准库类模板auto_ptr，将指针封装在auto_ptr之中，最终版代码由于使用了标准库auto_ptr类模板，代码大大简化了！

\3. auto_ptr类模板的使用可以在不增加代码量的情况下完美处理构造函数抛出异常的问题，auto_ptr类模板的设计目的正在于当指向动态分配的内存的指针本身停止活动(被销毁)时，所指内存被释放掉。

### 条款11：禁止异常（exceptions）流出destructors之外

1、两种情况下destructor会被调用：

(1)当对象在正常情况下被销毁，也就是当它离开了它的生存空间或是被明确的删除；

(2)当对象被exception处理机制——也就是exception传播过程中的stack-unwinding(栈展开)机制——销毁。

2、当destructor被调用时，可能(也可能不)有一个exception正在作用之中，但无法在destructor中区分这些状态(现在有了区分的办法。1995年7月 IOS/ANSI C++ 标准委员会加入一个新函数： uncaught_exception。如果某个exception正在作用中而尚未被捕捉的话，它会返回true)。当destructor抛出一个异常的时候而不会被destructor捕捉的时候，它会传播到destructor的调用端，如果这个destructor本身是因他某个exception而被调用的，terminate函数便会被调用，将程序结束掉，甚至不等局部对象被销毁，它默认调用abort函数终止程序。

\3. 对于2有一个解决办法可以防止destructor抛出的异常传出destructor之外：将destructor函数体包在一个try块之中，而catch(...)语句什么也不做。

\4. 除了2所示原因，让exception传出destructor之外还有一个坏处：如果destructor抛出exception而没有在当地捕捉，那个destructor就是执行不全（仅执行到抛出exception的那一点为止），也就是说它没有完成它应该完成的每一件事情。

\5. 综上，有两个理由支持我们采取措施阻止exception传出destructor之外：第一，它可以避免terminate函数在exception传播过程中的栈展开机制中被调用；第二，它可以协助确保destructors完成其应该完成的所有事情。

### 条款12：了解“抛出一个exception”与“传递一个参数”或“调用一个虚函数”之间的差异

 

### 条款13：以by refrence方式捕捉exceptions

 

### 条款14：明智运用exception specifications

 

 

### 条款15：了解异常处理（exception handing）的成本

 

 

## 效率（Efficiency）

### 条款16：谨记80-20法则

1、80-20法则:一个程序80%的资源用于20%的代码上(80%的执行时间花在大约20%的代码身上,80%的内存被大约20%的代码使用,80%的磁盘访问动作由20%的代码执行,80%的维护力气花在20%的代码上面).80-20法则的重点不在于字面上的数字,而是强调"软件的整体性能几乎总是由其构成要素(代码)的一小部分决定".

2、80-20法则说明软件性能的瓶颈往往只在一小部分代码,而由于"软件的性能特质倾向于高度的非直觉性",因而要找出性能瓶颈,一个有效的方法就是借助程序分析器(program profiler)测试程序各区段所使用资源.此外,为了防止测试数据的片面性,需要使用尽可能多的数据进行分析.

### 条款17：考虑使用lazy evaluation（缓式评估）

缓式评估其实就是拖延战术，直到逼不得已的时候才去计算。缓式评估的使用场景有：

1、引用计数，考虑String，String是一个内含char指针（char指针以'\0'结束）的资源管理类，正常情况下，String的copy构造和copy赋值都是深层copy，也就是对char指针指向的内容做一个副本，这个效率显然很低。考虑String s2 = s1; 后面的代码可能只是读取s2，没有必要做深层copy，s2与s1可以共享一份数据，也就是使用引用计数，来实现String。但是，当修改String的时候，必须做一个深层copy，也就是拖延战术。

2、区分读与写，考虑cout<<s2[1]与s2[1] = 'a'；为什么区分读和写？读取代价很小，写需要新建一个副本，因此需要区分。问题来了，如何区分呢？对String的数据char做一层封装，即CharProxy。对CharProxy的copy赋值就是写操作，增加隐式类型转换操作符operator char() const，就是读取操作。

3、缓式取出，考虑大型对象，内含很多个字段，为了保持一致性，这个大型对象存储在数据库。考虑需求，根据Id取出Name，如果把整个大型对象都读取出来，效率很差，因为有很多不使用的字段。更好的办法是，按需读取，需要什么取出什么。

4、表达式缓式评估，考虑两个大型矩阵相乘，运算量很大，但是我们往往只想要结果中的一个元素。因此，更好的解决办法是，相乘后不去运算，而是返回一个假的结果（代理结果）。读取结果中的一个元素时，也不去整个计算，而是只计算结果中的这一个元素。

缓式评估解决的问题是：避免不必要的计算。如果计算绝对需要，不能使用缓式评估。

### 条款18：分期摊还预期的计算成本

1、Over-eager evaluation("超急评估"):超前进度地做"要求以外"的更多工作,也就是在要求之前就把事情做下去.

2、Over-eager的背后观念是,如果预期程序常常会用到某个计算,可以降低每次计算的平均成本,方法就是设计一份数据结构以便能够极有效率地处理需求.

3、Over-eager evaluation实际上是一种"以空间换时间"的策略:Caching和Fetching都需要额外空间来(待)存放数据以降低访问所需时间.少数情况下,对象变大会降低软件性能:换页(paging)活动可能会增加,缓存击中率(cache hit rate)可能会降低.

over-eager evaluation和条款17的lazy evaluation并不矛盾:当必须支持某些运算但运算结果并不总是需要的时候,lazy evaluation可以改善程序效率;当必须支持某些运算且其结果总是被需要,或常常被需要的时候,over-eager evaluation可以改善程序效率.两者都比直接了当的eager evaluation难实现,但都可以为程序带来巨大的性能提升.

### 条款19：了解临时对象的来源

1、所谓的C++临时对象并不是程序员创建的用于存储临时值的对象,而是指编译器层面上的临时对象:这种临时对象不是由程序员创建,而是由编译器为了实现某些功能(例如函数返回,类型转换等)而创建.

  由于临时对象不是由程序员创建,其生存期由编译器掌控,因而也就不允许程序员对其进行更改,将其绑定到non-const 左值引用也就被禁止(见C++ 11: 右值引用,转移语义与完美转发),所有编译器都禁止将内置类型的non-const 左值引用绑定到内置类型的临时变量,但奇怪的是,有些编译器支持将类类型的non-const左值引用绑定到对应类类型的临时对象,比如vs.

2、临时对象通常在两种情况下被产生:一是当隐式类型转换在参数匹配时发生以使函数能够调用成功,二是函数返回对象的时候.了解这些临时对象那个何时被创建和销毁很重要,因为它们的构造和析构成本可能对程序性能造成影响.

3、临时对象可能很消耗资源,因此找出并(协助编译器)消除它们可能会给程序带来性能上的提升。

### 条款20：协助完成“返回值优化（RVO）”

1、函数如果返回对象,就会产生临时对象(见条款19)的构造,析构等过程。

2、大多数编译器具有一种优化方法—RVO(return value optimization,返回值优化),如果函数返回匿名对象,那么函数就有可能避免临时对象的构造。

3、要实现RVO,编译器通常要求函数返回匿名对象,新式的编译器支持其进化版—NRV(named return value),它可以对具名返回值做优化,消除构造和析构临时对象的成本。

其实RVO要求函数返回匿名对象是有理由的:如果函数返回匿名对象,那么就说明程序员只是利用该匿名对象存储返回值,而并不打算用该匿名对象做其他事情,因此编译器可以将其视为一个允许它做RVO优化的信号,而NRV优化虽然可以对具名返回值做优化,以消除匿名对象的成本,这提高了效率,但有可能是不是程序员的原意.

4、C++11引入了右值引用与转移语义,其目的不是消除临时对象的构造和析构,而是通过右值引用绑定到右值来延长临时对象生存期,从而直接使用临时对象,这种方法与RVO异曲同工:前者是直接使用临时对象,不再构造新对象;后者避免构造临时对象,将本应构造在临时对象上的内容直接构造到新对象上.

### 条款21：利用重载技术（overload）避免隐式类型转换（implicit type conversions）

使用函数重载来避免隐式类型转换所产生的临时对象，从而提高程序运行效率不仅仅局限运用在操作符函数身上。在大部分程序中，如果可以接受一个char*,你可能会希望也接受一个string对象。

### 条款22：考虑以操作符复合形式（op=）取代其独身形式（op）

对于内置类型，x = x+y 与x+=y的结果相同。 x=x+y 与 x+=y的结果相同，但二者做的事情差别很大。

　　a、x=x+y做的事情：方法内有个局部对象，值为x+y，返回局部对象，返回值是个临时对象，这个临时对象赋值给x。

b、x+=y做的事情：直接在x上操作，修改x的内容，并返回x的引用。

操作符的复合形式通常比其独身形式效率更高:独身形式需要返回新对象,因而需要承担临时对象的构造和析构成本,复合形式直接将结果写入其左端自变量,不需要产生临时对象放置返回值。’

const Rational operator+(const Rational& lhs, const Rational& rhs){

  Rational temp(lhs);

  temp += rhs;

  return temp;

}

### 条款23：考虑使用其他程序库

不同的程序库即使提供相似的机能，也往往表现出不同的性能取舍策略，所以一旦你找出程序的瓶颈，你应该思考是否有可能因为改用另一个程序库而移除了那些瓶颈。如果你的程序有一个I/0瓶颈，你可以考虑以stdio代替iostream，但如果你花太多的时间在动态内存分配和释放方面，你或许应该看看是否有其他提供了operator new和operator delete的程序库。

由于不同的程序库将效率、扩充性、移植性、类型安全性等不同设计具体化，有时候可以找找看是否存在另一个功能相近的程序库而其在效率上较高的设计权重，如果有，改用它。

### 条款24：了解virtual functions、multiple inheritance、virtual base classes、runtime type identification的成本

C++要支持面向对象,付出了一定的时间和空间成本,但是也因此实现了更强大易用的功能.如果坚持使用C语言,那么以上功能都必须自己打造.相对于编译器产生的代码,自己打造的东西可能比较没效率,也不够鲁棒性.从效率上来说,自己动手打造未必会比编译器做的更好.当然,有时确实要避免编译器在背后所做的这些工作,比如隐藏的vptr以及"指向virtual base classes"的指针,可能会造成"将C++对象存储于数据库"或"在进程边界间搬移C++对象"时的困难度提高,这是可能就需要手动模拟这些性质.

## 技术（Techniques，Idioms，Patterns）

### 条款25：将constructor和non-member function虚化

原则上构造函数不能为虚:虚函数用于实现"因类型而异的行为",也就是根据指针或引用所绑定对象的动态类型而调用不同实体,但构造函数用于构造对象,在对象构造之前自然没有动态类型的概念,虚与非虚也就无从谈起.所谓的的virtual-constructor实际上是"仿virtual-constructor",它本质上不是constructor,但能够产生不同类型的对象,从而实现"virtual-constructor"的功能。

正如constructors无法被虚化,non-member function原则上也无法被虚化，non-member的虚化和virtual copy constructor的实现类似:写一个虚函数做实际工作,再一个非虚函数调用该虚函数.此外,为了避免函数调用的额外成本(毕竟operator<<只有一句,不值得将压弹栈的成本),可以将非虚函数inlin化.

### 条款26：限制某个class所能产生的对象

\1. 针对某些有特殊功能的类,有时需要限制其对象数量,例如系统中只有一个打印机,因此需要将打印机的对象数目限制为1,或者只有16个file descriptor(文件描述器)可用,因此必须确定不会有更多的descriptor objects被产生出来,在这些情况下.就需要限制对象数目,或者说阻止对象被产生出来.

\2. 允许零个或一个对象。要限制对象数目,最直接的策略是限制构造函数的调用(以下称策略1),采用这种思想将构造函数设为private,然后声明一个友元函数调用它,并生成一个static对象,

\3. 一个用来计算对象个数的Base Classes。可以将2中的策略1和策略2的结合一般化,将它抽象为一个类模板,任何需要限制对象数目的类只要继承这个模板的实例化即可。

### 条款27：要求（或禁止）对象产生于heap之中

\1.   要求对象产生于堆中（只能用new）

\2.   判断某个对象是否位于heap内

\3.   禁止对象产生于heap之中

### 条款28：Smart Pointers（智能指针）

 

### 条款29：Reference counting（引用计数）

 

### 条款30：Proxy classes（替身类、代理类）

 

### 条款31：让函数根据一个以上的对象类型来决定如何虚化

 

 

## 杂项讨论：

### 条款32：在未来时态下发展程序

\1. 所谓"在未来时态下发展程序",指的是是程序需要具有良好的可扩展性和可维护性,它要求程序:功能齐全,接口易用,代码泛化,以下原则有助于实现这一目标:

  1). 以C++本身表现各种规范而不是仅仅依赖于注释:如果某个class不打算作为基类,那么就应该以C++语法阻止派生(条款26);如果一个class要求所有对象实体在heap内产生,就以C++语法严格厉行这项约束(条款27),如果copying和assignment对某个class没有意义,就声明为private...

  2). 如果某个类可能会被派生,就将析构函数声明为virtual,如果某个函数在派生类中可能会被重定义,就把它声明为virtual.

  3). 为每一个class处理assignment和copy construction,如果这些函数不易完成,就把它们声明为private以防止编译器自行合成错误版本(条款E11).

  4). 使class的操作符核函数拥有自然的语法和直观的语义,和内置类型行为保持一致.

  5). 任何事情只要能做,就会有人做.如自我赋值,初始化前使用对象,给对象超出范围的值等.因此要使程序具有健壮性,有预防,侦测,甚至更正的能力.

  6). 尽量写可移植代码.

  7). 尽量降低代码之间的依赖性,将更改系统产生的影响降到最小:尽量采用封装性质(条款E20);尽量用匿名namespaces或文件内的static对象和static函数;尽量避免设计virtual base class,因为它的初始化需要由底层派生类完成;尽量避免使用RTTI而导致一层一层的if-else;尽量使用类声明胃不是类定义以降低文件之间的编译依赖性.

\2. 当然,"现在式思维"也很重要:平台依赖,开发时间限制,性能考虑等因素都是重要的约束.但在条件允许的情况下应该使用"未来式思维"编程.

### 条款33：将非尾端类（non-leaf classes）设计为抽象类（abstract classes）

当具体类被当做基类使用时,应该将具体类转变为抽象基类。

![img](file:///C:/Users/原来是~1/AppData/Local/Temp/msohtmlclip1/01/clip_image002.jpg) ![img](file:///C:/Users/原来是~1/AppData/Local/Temp/msohtmlclip1/01/clip_image004.jpg)

然而有时候需要使用第三方库,并继承其中一个具体类,由于无法修改该库,也就无法将该具体类转为抽象基类,这是就需要采取其他选择:

  1). 继承自现有的具体类,但要注意1所提出的assignment问题,并小心条款3所提出的数组陷阱.

  2). 试着在继承体系中找一个更高层的抽象类,然后继承它.

  3). 以"所希望继承的那么程序库类"来实现新类.例如使用复合或private继承并提供相应接口.此策略不具灵活性.

  4). 为"所希望继承的那么程序库类"定义一些non-member,不再定义新类.

### 条款34：如何在同一个程序中结合C++和C

如果你打算在同一程序中混合使用C++和C，请记住以下简单的守则：

·确定你的C++和C编译器产出兼容的目标文件（object files）。

·将双方都使用的函数声明为extern “C”。

·如果可能，尽量在C++中撰写main。

·总是以delete删除new返回的内存；总是以free释放malloc返回的内存。

·将两个语言间的“数据结构传递”限制于C所能了解的形式；C++ structs如果内含非虚函数，倒是不受此限制。

### 条款35：让自己习惯于标准C++语言

\1. 1990年后C++的重要改变

  1). 增加了新的语言特性:RTTI，namespaces，bool,关键词mutable和explicit，enums作为重载函数之自变量所引发的类型晋升转换,以及"在class 定义区内直接为整数型(intergral) const static class members设定初值"的能力.

   2). 扩充了Templates的特性:允许member templates存在,接纳"明白只是template当场实例化"的标准语法,允许function templates接受"非类型自变量(non-type atguments)",可用class templates作为其他template的自变量.

  3). 强化了异常处理机制(Exception handling):编译期间更严密地检验exception specifications,允许unexcpted函数抛出bad_exception对象.

  4).修改了内存分配例程:假如operator new[ ]和operator delete[ ],内存未能分配成功时由operator new/new[ ]抛出一个异常,提供一个operator new/new[ ]新版本,在内存分配失败后返回0.

  5). 增加了新的转型形式:static_cast，dynamic_cast，const_cast和reinterpret_cast

\2. 标准程序库的能力

  1). 支持C标准函数库.

  2). 支持strings.

  3). 支持国别(本土化,localization).不同文化使用不同字符集以及不同的日期,时间,排序事物,货币值等显式习俗.

  4). 支持I/O.

  5). 支持数值应用.支持复数和数组类,提供常用函数,包括"部分和(partial sum)"以及"相邻差值(adjacent difference)".

\3. 标准库特点

  1). 高度模板化——每一样东西几乎都是template.例如string是basic_string<char>的typedef(basic_string还可支持wide char，unicode char)，IOstreams也是template,它有个类型参数(type parameter)用来定义streams(数据流)的字符类型.

  2). 所有成分都位于namespace std内.

\4. Standart Template Library(STL)

  1). STL占据了C++标准库的大部分,主要包括三部分:容器(container),迭代器(iterator),泛型算法(algorithm).其中容器持有对象,迭代器用于遍历容器元素,泛型算法则基于迭代器实现不依赖于具体类型的函数模板.

  2). STL是可扩充的,只要遵循STL的标准,可以将自己的容器,迭代器,算法等结合STL使用.(要使自定义的迭代器适用于STL的泛型算法,需要了解C++的traits技法,见Effective C++ 条款47)