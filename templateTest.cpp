#include <iostream>
#include <string>
//将参数名字符串化
#define Name(X) #X
//定义输出函数，打印为"参数名： 参数值"
#define Out(X) {string xname=Name(X);cout<<xname.substr(2,xname.size()-1)<<": "<<X<<endl;}

using namespace std;

template <typename T>
class paraPrint
{
public:

	paraPrint(T paras)
	{
		this->paras = paras;
	}

	void parasOut()
	{
		cout << paras << endl;
	}

private:

	T paras;
};