#include "Use_AStar.h"
#include<iostream>
#include<stack>
#include<algorithm> 
using namespace Use_AStar;
void Use_AStar::Init(Generator& generator)
{
	// Set 2d map size.
	generator.setWorldSize({15, 11});
	//添加障碍 
	generator.addCollision({1,1});
	generator.addCollision({1,3});
	generator.addCollision({1,5});
	generator.addCollision({1,7});
	generator.addCollision({1,9});
	generator.addCollision({4,1});
	generator.addCollision({4,3});
	generator.addCollision({4,5});
	generator.addCollision({4,7});
	generator.addCollision({4,9});
	generator.addCollision({7,1});
	generator.addCollision({7,3});
	generator.addCollision({7,5});
	generator.addCollision({7,7});
	generator.addCollision({7,9});
	generator.addCollision({10,1});
	generator.addCollision({10,3});
	generator.addCollision({10,5});
	generator.addCollision({10,7});
	generator.addCollision({10,9});
	generator.addCollision({13,1});
	generator.addCollision({13,3});
	generator.addCollision({13,5});
	generator.addCollision({13,7});
	generator.addCollision({13,9});
	
	// You can use a few heuristics : manhattan, euclidean or octagonal.
	generator.setHeuristic(AStar::Heuristic::manhattan);
	//设置是否可以走对角线 
}

//获取从source_到target_的路径 
CoordinateList Use_AStar::findthePath(Vec2i source_,Vec2i target_,Generator& generator)
{
	CoordinateList direction={{-1,0},{1,0},{0,-1},{0,1}};//上下左右四个方向
	int minTime=INT_MAX;
	CoordinateList ans;
	for(int i=0;i<4;i++)
	{
		Vec2i node;
		node.x=direction[i].x+target_.x;
		node.y=direction[i].y+target_.y;
		
		CoordinateList path=generator.findPath(source_,node,(i<2?1:0));
		int time=generator.calculateTimeWasted(path);
		if(time<minTime)
		{
			minTime=time;
			ans=path;
		}
	}
	return ans;
}

//获取这条路径上所用的时间
int Use_AStar::getWaistedTime(CoordinateList path,Generator& generator)
{
	return generator.calculateTimeWasted(path)*2+2+0.5+1;
}
//读取data.txt文件 
vector<int>& Use_AStar::readFile()
{
	freopen("data.txt","r",stdin);
	int x;
	static vector<int> ans;
	while(cin>>x)
	{
		ans.push_back(x);
	}
	return ans;
}
vector<Vec2i>& Use_AStar::getTargets(vector<int> node)
{
	static vector<Vec2i> nodes; //根据地图获取各个节点的位置 
	for(int i=0;i<node.size();i++)
	{
		int num=node[i];
		int x;
		switch(num%5)
		{
			case 1:x=1;break;
			case 2:x=4;break;
			case 3:x=7;break;
			case 4:x=10;break;
			case 0:x=13;break;
		}
		int y;
		switch((num-1)/5)
		{
			case 0:y=9;break;
			case 1:y=7;break;
			case 2:y=5;break;
			case 3:y=3;break;
			case 4:y=1;break;
		}
		nodes.push_back({x,y});
	}
	return nodes;
}
vector<Car>& Use_AStar::run(Generator& generator)
{
	//开始运行
	
//	cout<<"开始运行..."<<endl; 
	
	vector<int> node=readFile();//获取节点列表
	
//	cout<<"获取到了节点列表"<<endl;
	
	vector<Vec2i> nodes; //根据地图获取各个节点的位置 
	nodes=getTargets(node);
	
//	cout<<"获取到了节点位置"<<endl;
	
	double nowTime=0;//当前时间
	stack<Car> cars;//小车
	static vector<Car> leftcars;//存储已经离开的小车
	for(int i=8;i>=1;i--)
	{
		Car car=*new Car();
		car.num=i;
		car.time=2.5*(9-i);//每辆车的初始出发时间不同
//		cout<<"正在添加小车...编号为"<<i<<endl;
		cars.push(car);
	}
	
//	cout<<"初始化了小车"<<endl; 
	
	//当剩余货物不为空
	while(!nodes.empty()){
		
//		cout<<"正在将剩余的货物添加到小车上"<<endl;
		
		Vec2i currentnode=nodes[0];
		
//		cout<<"此货物位置为："<<currentnode.x<<","<<currentnode.y<<endl;
		
		if(cars.empty())//剩余车辆为空
		{
			//所有小车都出去了，在此处根据所有小车的路径判断碰撞的发生
			int crash_table[15][11];
			for(int i=0;i<15;i++)//inite
				for(int j=0;j<11;j++)
					crash_table[i][j]=0;
			//建立一张二维碰撞表，记录在2秒内所有小车的位置信息，据此判断是否会有碰撞产生
			//判断依据：获得每一秒在地图中的小车位置，若地图中某位置有小车，碰撞表中相应的值+1，若没有，则为0 
			int minpath=INT_MAX;
			for(int i=0;i<leftcars.size();i++)
			{
				if(leftcars[i].paths.size()<minpath)//有更少的路
				{
					minpath=leftcars[i].paths.size();
				}
			}
//			cout<<"剩余小车不足，添加新货物"<<endl; 
			
			//获得最先回来的车辆下标 
			double minTime=leftcars[0].time;
			int index=0;
			for(int i=1;i<leftcars.size();i++)
			{
				if(leftcars[i].time<minTime)
				{
					minTime=leftcars[i].time; 
					index=i;
				}
			}
			
//			cout<<"获取到了最先回来的小车"<<index+1<<endl;
			
			cars.push(leftcars[index]);//回来了 
			leftcars.erase(leftcars.begin()+index);//去除最先回来的一个节点
		}
		nodes.erase(nodes.begin());//删除顶部货物
		
//		cout<<"添加了新货物"<<endl;
		
		Car car=cars.top();//获得顶部的小车
		cars.pop();//小车出去工作
//		cout<<"有一辆小车出去工作了,时间："<<car.time<<" 编号："<<car.num<<endl;
		
		car.addPath(currentnode,findthePath({7,10},currentnode,generator),generator);//小车添加路径
		
//		cout<<"这辆小车完成了工作,时间："<<car.time<<" 编号："<<car.num<<endl;
		
		leftcars.push_back(car);//加入到leftcars
	}
	
	
	//最后获得的为leftcars及其附带的路径、时间等信息 
	//进行显示相关操作
	
	cout<<"编号\t";
	cout<<"1\t2\t3\t4\t5\t6\t7\t8\n";
	cout<<"时间\t";
	for(int i=0;i<leftcars.size();i++)
	{
		cout<<leftcars[i].time<<"\t";
	}
	cout<<"\n";
	return leftcars;
	//下面估计碰撞损耗时间
	/* 
	cout<<"回来的时间列表:\n";
	for(int i=0;i<leftcars.size();i++)
	{
		for(int j=0;j<leftcars[i].Times.size();j++)
		cout<<leftcars[i].Times[i]<<" ";
		
	}
	*/ 
}

void Car::addPath(Vec2i target_,CoordinateList path,Generator& generator)
{
	paths.push_back(findthePath({7,10},target_,generator));
	//完成时间=路径上花费的时间（包括转弯）+装货、扫码、卸货时间+避开障碍时间（待求） 
	time+=getWaistedTime(paths[paths.size()-1],generator);//当前完成时间 
	Times.push_back(time);//每一辆机器人车的完成任务的时间序列都保存下来 
}
