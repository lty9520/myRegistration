#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ����ͷ�ļ���
#include <Eigen/src/StlSupport/details.h>
#include <pcl/registration/icp.h>
#include <pcl/common/time.h>

using namespace std;
using namespace pcl;




//01�Ӷ�
struct com01
{
	double com01_b;
	double com01_b1;
	double com01_b2;
	double com01_h;
	double com01_h1;
	double com01_h2;
	double com01_h3;	//**inner**
	double com01_t;
	double com01_t1;
	double com01_t2;	//**inner**
};

//02�����
struct com02
{
	double com02_b;
	//double com02_b1 = com01_b2;
	double com02_b2;
	double com02_h;
	double com02_h1;	//**inner**
	double com02_h2;
	//double com02_t = com01_t1;
	int m;
};

//03����
struct com03
{
	double com03_b;
	//double com03_b1 = com01_t1;
	double com03_b2;
	double com03_b3;	//**inner**
	//double com01_h = com02_h;
	double com03_h1;
	double com03_h2;	//**inner**
	double com03_h3;	//**inner**
	//double com03_t = com01_b2;
	int m;
};

//04ɢ��
struct com04
{
	double com04_b;
	double com04_b1;
	double com04_b2;
	double com04_h;
	double com04_h1;
	double com04_h2;
	double com04_t;
};

//05������
struct com05
{
	double com05_b;
	double com05_b1;
	double com05_b2;
	double com05_h;
	double com05_h1;
	double com05_h2;
	double com05_t;
	double com05_t1;
};

//06������
struct com06
{
	double com06_b;
	//double com06_b1 = com05_t1;
	double com06_b2;
	double com06_h;
	double com06_h1;	//**inner**
	double com06_h2;
	//double com06_t = com04_b2;
	int m;
};

//07���Ӗ�
struct com07
{
	double com07_b;
	//double com07_b1 = com05_t1;
	double com07_b2;
	//double com07_h = com06_h;
	double com07_h1;	//**inner**
	double com07_h2;
	//double com07_t = com05_b2;
	int m;
};

//08����-2
struct com08
{
	double com08_b;
	double com08_b1;
	double com08_b2;
	//double com08_b3 = com05_b2;
	double com08_b4;
	//double com08_b5 = com04_b2;
	//double com08_h = com06_h;
	double com08_h1;
	double com08_h2;	//**inner**
	double com08_h3;	//**inner**
	//double com08_t = com05_t1;
	int m;
};

//09��ͷ��
struct com09
{
	double com09_b;
	//double com09_b1 = com05_t1;
	double com09_h;
	double com09_h1;	//**inner**
	//double com09_t = com04_b2;
};

//10����
struct com10
{
	double com10_b;
	//double com10_b1 = com05_t1;
	double com10_b2;
	//double com10_h = com09_h;
	double com10_h1;	//**inner**
	double com10_h2;
	//double com10_t = com04_b2;
	int m;
};

//11���
struct com11
{
	double com11_b;
	//double com11_b1 = com05_t1;
	double com11_b2;
	//double com11_h = com09_h;
	double com11_h1;	//**inner**
	double com11_h2;
	//double com11_t = com05_b2;
	int m;
};

//12ˣͷ
struct com12
{
	double com12_b;
	double com12_b1;
	double com12_b2;
	//double com12_b3 = com05_b2;
	double com12_b4;
	//double com12_b5 = com04_b2;
	double com12_b6;
	//double com12_b7 = com04_b2;
	//double com12_h = com09_h;
	double com12_h1;
	double com12_h2;	//**inner**
	double com12_h3;	//**inner**
	double com12_h4;	//**inner**
	//double com12_t = com05_t1;
};

//13��ͷ��2
struct com13
{
	double com13_b;
	//double com13_b1 = com05_t1;
	double com13_h;
	double com13_h1;	//**inner**
	//double com13_t = com04_b2;
};

//14�̰���
struct com14
{
	double com14_b;
	//double com14_b1 = com05_t1;
	//double com14_h = com13_h;
	double com14_h1;	//**inner**
	//double com14_t = com04_b2;
};

//15��ͷľ
struct com15
{
	double com15_b;
	double com15_b1;
	//double com15_b2 = com05_b2;
	double com15_b3;
	//double com15_b4 = com04_b2;
	double com15_b5;
	//double com15_b6 = com04_b2;
	double com15_b7;
	//double com15_b8 = com04_b2;
	//double com15_h = com13_h;
	double com15_h1;	//**inner**
	double com15_h2;	//**inner**
	double com15_h3;	//**inner**
	double com15_h4;	//**inner**
	//double com15_t = com05_t1;
};



/*
	parasOut �� ���������Ľṹ��
	/
	|	com01 �� 01�Ӷ�
	|	com02 �� 02�����
	|	com03 �� 03����
	|	com04 �� 04ɢ��
	|	com05 �� 05������
	|	com06 �� 06������
	/	com07 �� 07���Ӗ�
	\	com08 �� 08����-2
	|	com09 �� 09��ͷ��
	|	com10 �� 10����
	|	com11 �� 11���
	|	com12 �� 12ˣͷ
	|	com13 �� 13��ͷ��2
	|	com14 �� 14�̰���
	|	com15 �� 15��ͷľ
	\
*/
struct parasOut
{
	vector<pair<string, double>> paras01;
	vector<pair<string, double>> paras02;
	vector<pair<string, double>> paras03;
	vector<pair<string, double>> paras04;
	vector<pair<string, double>> paras05;
	vector<pair<string, double>> paras06;
	vector<pair<string, double>> paras07;
	vector<pair<string, double>> paras08;
	vector<pair<string, double>> paras09;
	vector<pair<string, double>> paras10;
	vector<pair<string, double>> paras11;
	vector<pair<string, double>> paras12;
	vector<pair<string, double>> paras13;
	vector<pair<string, double>> paras14;
	vector<pair<string, double>> paras15;
};

/*
	calcParas �� �м��������ṹ��
	/
	|	bboxQ �� ��Ԫ��(rotation of bouding box)
	/	bboxT �� ��ά����(translation of bouding box)
	\	cenpot �� ���ĵ�
	|	pcaX��pcaY��pcaZ �� �������
	|	whd(width&height&depth) �� ��ά����	
	\
*/
struct parasCalc
{
	//��ת����
	Eigen::Quaternionf bboxQ;
	//ƽ�ƾ���
	Eigen::Vector3f bboxT;
	pcl::PointXYZ cenpot, pcaX, pcaY, pcaZ;
	Eigen::Vector3f whd;
	float sc;
};

/*
	myModel �� ģ������ṹ��
	/
	|	cloud �� pcl::PointXYZ��ʽ��������
	/	transformedCloud �� pcl::PointXYZ��ʽ��������
	\	calcP �� �м����ṹ��
	|	outP �� ���������ṹ��
	|	file �� �����ļ�
	\
*/
struct myModel
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud;
	parasCalc calcP;
	parasOut outP;
	string file;
};

void split(const string& s, vector<pair<string, double>>& sv, const char flag = ' ') {
	//sv.clear();
	istringstream iss(s);
	string temp;
	vector<string> tempv;
	while (getline(iss, temp, flag)) {
		tempv.push_back(temp);
	}
	pair<string, double> tempp;
	if (tempv.size() < 2)
	{
		tempp.first = tempv[0];
		tempp.second = 0;
	}
	else 
	{
		tempp.first = tempv[0];
		tempp.second = stod(tempv[1]);
	}
	
	sv.push_back(tempp);
	return;
}



int main()
{
	//myModel temp = { 10, 0.05, "aaa" };
	myModel model;
	cout << "print out :" << endl;
	//cout << temp << endl;

	vector<pair<string, double>>paras;
	string line;
	ifstream open("shuchu.txt");
	while (getline(open, line))
	{
		split(line, paras, '\t');
		
	}
	open.close();


	vector < pair<string, vector<pair<string, double>> >> outP;
	int n = 0;
	string namefir = "component";
	stringstream name;
	pair<string, vector<pair<string, double>> >temp;
	vector<pair <string, double>>tempv;
	for (int i = 0; i < paras.size(); i++)
	{
		
		if (paras[i].first == "b")
		{
			if (temp.first == "" && temp.second.size() == 0)
			{
				n++;
				name << namefir << n;
				temp.first = name.str();
				tempv.clear();
				tempv.push_back(paras[i]);
			}
			else
			{
				temp.second = tempv;
				outP.push_back(temp);
				n++;
				name.str("");
				name << namefir << n;
				temp.first = name.str();
				tempv.clear();
				tempv.push_back(paras[i]);
				
			}
			
		}
		else
		{
			tempv.push_back(paras[i]);
		}
	}

	for (int i = 0; i < paras.size(); i++)
	{
		cout << paras[i].first << " = " << paras[i].second << endl;
	}

	system("pause");
}