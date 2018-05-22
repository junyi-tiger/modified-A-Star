#include <iostream>
#include "AStar.h"
#include "Use_AStar.h" 
using namespace Use_AStar;
using namespace AStar;
int main()
{
	AStar::Generator generator;
    //进行初始化
    cout<<"正在初始化......\n";
	Init(generator);
	
//	CoordinateList thisone=findthePath({7,10},{1,1},generator);
//	generator.printPath(thisone);
	
	
	//打印初始地图
	cout<<"打印初始地图......\n";
	generator.printMap();
	std::cout << "程序正在运行......\n";
	vector<Car> paths=run(generator);
	//下面输出时间序列 
	for(int i=0;i<paths.size();i++)
	{
		cout<<"第"<<i+1<<"个机器人运货时每次运货的完成时间如下:(总运货次数："<<paths[i].Times.size()<<")"<<endl; 
		for(int j=0;j<paths[i].Times.size();j++)
		{
			cout<<paths[i].Times[j]<<" ";
		}
		cout<<endl;
	 } 
	//下面输出第一辆机器人的路径 将path[0]改为path[1],path[2],...可输出第二辆,第三辆...机器人的路径
	cout<<"第一辆小车路径如下："<<endl;
	for(int i=0;i<paths[0].paths.size();i++)
	{
		generator.printPath(paths[0].paths[i]);
	}
	system("pause");
	return 0;
}
