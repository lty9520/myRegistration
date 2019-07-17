#include <iostream>
#include <string>

using namespace std;

struct MyStruct
{
	int i;
	double d;
	string s;

};

int main()
{
	MyStruct temp = { 10, 0.05, "aaa" };

	cout << "print out :" << endl;
	cout << temp << endl;

	system("pause");
}