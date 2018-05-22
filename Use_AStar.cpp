#include "Use_AStar.h"
#include<iostream>
#include<stack>
#include<algorithm> 
using namespace Use_AStar;
void Use_AStar::Init(Generator& generator)
{
	// Set 2d map size.
	generator.setWorldSize({15, 11});
	//����ϰ� 
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
	//�����Ƿ�����߶Խ��� 
}

//��ȡ��source_��target_��·�� 
CoordinateList Use_AStar::findthePath(Vec2i source_,Vec2i target_,Generator& generator)
{
	CoordinateList direction={{-1,0},{1,0},{0,-1},{0,1}};//���������ĸ�����
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

//��ȡ����·�������õ�ʱ��
int Use_AStar::getWaistedTime(CoordinateList path,Generator& generator)
{
	return generator.calculateTimeWasted(path)*2+2+0.5+1;
}
//��ȡdata.txt�ļ� 
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
	static vector<Vec2i> nodes; //���ݵ�ͼ��ȡ�����ڵ��λ�� 
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
	//��ʼ����
	
//	cout<<"��ʼ����..."<<endl; 
	
	vector<int> node=readFile();//��ȡ�ڵ��б�
	
//	cout<<"��ȡ���˽ڵ��б�"<<endl;
	
	vector<Vec2i> nodes; //���ݵ�ͼ��ȡ�����ڵ��λ�� 
	nodes=getTargets(node);
	
//	cout<<"��ȡ���˽ڵ�λ��"<<endl;
	
	double nowTime=0;//��ǰʱ��
	stack<Car> cars;//С��
	static vector<Car> leftcars;//�洢�Ѿ��뿪��С��
	for(int i=8;i>=1;i--)
	{
		Car car=*new Car();
		car.num=i;
		car.time=2.5*(9-i);//ÿ�����ĳ�ʼ����ʱ�䲻ͬ
//		cout<<"�������С��...���Ϊ"<<i<<endl;
		cars.push(car);
	}
	
//	cout<<"��ʼ����С��"<<endl; 
	
	//��ʣ����ﲻΪ��
	while(!nodes.empty()){
		
//		cout<<"���ڽ�ʣ��Ļ�����ӵ�С����"<<endl;
		
		Vec2i currentnode=nodes[0];
		
//		cout<<"�˻���λ��Ϊ��"<<currentnode.x<<","<<currentnode.y<<endl;
		
		if(cars.empty())//ʣ�೵��Ϊ��
		{
			//����С������ȥ�ˣ��ڴ˴���������С����·���ж���ײ�ķ���
			int crash_table[15][11];
			for(int i=0;i<15;i++)//inite
				for(int j=0;j<11;j++)
					crash_table[i][j]=0;
			//����һ�Ŷ�ά��ײ����¼��2��������С����λ����Ϣ���ݴ��ж��Ƿ������ײ����
			//�ж����ݣ����ÿһ���ڵ�ͼ�е�С��λ�ã�����ͼ��ĳλ����С������ײ������Ӧ��ֵ+1����û�У���Ϊ0 
			int minpath=INT_MAX;
			for(int i=0;i<leftcars.size();i++)
			{
				if(leftcars[i].paths.size()<minpath)//�и��ٵ�·
				{
					minpath=leftcars[i].paths.size();
				}
			}
//			cout<<"ʣ��С�����㣬����»���"<<endl; 
			
			//������Ȼ����ĳ����±� 
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
			
//			cout<<"��ȡ�������Ȼ�����С��"<<index+1<<endl;
			
			cars.push(leftcars[index]);//������ 
			leftcars.erase(leftcars.begin()+index);//ȥ�����Ȼ�����һ���ڵ�
		}
		nodes.erase(nodes.begin());//ɾ����������
		
//		cout<<"������»���"<<endl;
		
		Car car=cars.top();//��ö�����С��
		cars.pop();//С����ȥ����
//		cout<<"��һ��С����ȥ������,ʱ�䣺"<<car.time<<" ��ţ�"<<car.num<<endl;
		
		car.addPath(currentnode,findthePath({7,10},currentnode,generator),generator);//С�����·��
		
//		cout<<"����С������˹���,ʱ�䣺"<<car.time<<" ��ţ�"<<car.num<<endl;
		
		leftcars.push_back(car);//���뵽leftcars
	}
	
	
	//����õ�Ϊleftcars���丽����·����ʱ�����Ϣ 
	//������ʾ��ز���
	
	cout<<"���\t";
	cout<<"1\t2\t3\t4\t5\t6\t7\t8\n";
	cout<<"ʱ��\t";
	for(int i=0;i<leftcars.size();i++)
	{
		cout<<leftcars[i].time<<"\t";
	}
	cout<<"\n";
	return leftcars;
	//���������ײ���ʱ��
	/* 
	cout<<"������ʱ���б�:\n";
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
	//���ʱ��=·���ϻ��ѵ�ʱ�䣨����ת�䣩+װ����ɨ�롢ж��ʱ��+�ܿ��ϰ�ʱ�䣨���� 
	time+=getWaistedTime(paths[paths.size()-1],generator);//��ǰ���ʱ�� 
	Times.push_back(time);//ÿһ�������˳�����������ʱ�����ж��������� 
}
