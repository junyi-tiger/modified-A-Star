#include <iostream>
#include "AStar.h"
#include "Use_AStar.h" 
using namespace Use_AStar;
using namespace AStar;
int main()
{
	AStar::Generator generator;
    //���г�ʼ��
    cout<<"���ڳ�ʼ��......\n";
	Init(generator);
	
//	CoordinateList thisone=findthePath({7,10},{1,1},generator);
//	generator.printPath(thisone);
	
	
	//��ӡ��ʼ��ͼ
	cout<<"��ӡ��ʼ��ͼ......\n";
	generator.printMap();
	std::cout << "������������......\n";
	vector<Car> paths=run(generator);
	//�������ʱ������ 
	for(int i=0;i<paths.size();i++)
	{
		cout<<"��"<<i+1<<"���������˻�ʱÿ���˻������ʱ������:(���˻�������"<<paths[i].Times.size()<<")"<<endl; 
		for(int j=0;j<paths[i].Times.size();j++)
		{
			cout<<paths[i].Times[j]<<" ";
		}
		cout<<endl;
	 } 
	//���������һ�������˵�·�� ��path[0]��Ϊpath[1],path[2],...������ڶ���,������...�����˵�·��
	cout<<"��һ��С��·�����£�"<<endl;
	for(int i=0;i<paths[0].paths.size();i++)
	{
		generator.printPath(paths[0].paths[i]);
	}
	system("pause");
	return 0;
}
