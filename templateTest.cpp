#include <iostream>
#include <string>
//���������ַ�����
#define Name(X) #X
//���������������ӡΪ"�������� ����ֵ"
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