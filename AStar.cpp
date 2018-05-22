#include "AStar.h"
#include <algorithm>
#include<iostream>
using namespace std::placeholders;
//拷贝函数+运算符重载
bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

//节点相加 便于方向的控制
AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

//Node构造函数
AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

//启发函数 
AStar::uint AStar::Node::getScore()
{
    return G + H;
}

//默认构造函数 
AStar::Generator::Generator()
{
    setHeuristic(&Heuristic::manhattan);
    //方向的优先选取：优先的放到后面 
    direction1 = {
    	//当终点位置位于卸货位置的左、右边时  type=1
    	//寻路时优先选取横向的方向
    	//左、右、上、下
    	{ 0, 1 },{ 0, -1 }, { -1, 0 },{ 1, 0 },
    };
    direction2 = {
    	// 当终点位置位于卸货位置的上、下边时 type=0
    	//寻路时优先选取纵向的方向
    	 //上、下、左、右 
		{ -1, 0 }, { 1, 0 }, { 0 , -1 }, { 0, 1 },
	};
}

//设置地图大小 
void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

//设置4方向运动还是八方向 

//设置启发函数类型 
void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

//添加障碍 
void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

//移除障碍 
void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

//清空障碍 
void AStar::Generator::clearCollisions()
{
    walls.clear();
}

//寻找一条从source_到target_的路径，返回寻找到的路径的栅格列表 
AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_,int type)//type表明终点类型 
{
	//A star算法核心
    Node *current = nullptr;
    NodeVec openSet; 
    NodeSet closedSet;
	openSet.insert(openSet.begin(),new Node(source_));
	CoordinateList ans;
	int minTimeWasted=INT_MAX;
	//当开启集不为空时 
    while (!openSet.empty()) {
        current = openSet[0];//获得开启集的第一个
        for (auto node : openSet) {//获得最小代价的节点，赋值为current 
            if (node->getScore() <= current->getScore()) {
                current = node;
            }
        }

        if (current->coordinates == target_) {//到了目的地
        	break; 
        	/*
        	//计算总时间代价
        	CoordinateList path;
        	Node *temp=current;
			while (temp != nullptr) {
			path.push_back(temp->coordinates);
        	temp = temp->parent;
    		}
    		int time=calculateTimeWasted(path);
    		if(time<minTimeWasted){
    			minTimeWasted=time;
    			ans=path;
			}
			openSet.erase(std::find(openSet.begin(), openSet.end(), current));//从openSet中删除此current节点 
        	if(openSet.empty())//为空的话，结束搜寻 
            break;
            //不为空，继续搜寻 
			continue; 
			*/
        }

        closedSet.insert(current);//将current加入到关闭集
        openSet.erase(std::find(openSet.begin(), openSet.end(), current));//将current从开启集中删除

        for (uint i = 0; i < 4; ++i) {
        	Vec2i newCoordinates;
        	if(type)//终点在卸货位置的左右边
            newCoordinates=current->coordinates + direction1[i];//按顺序获得四个方向中某个方向上的栅格位置
            else newCoordinates=current->coordinates + direction2[i];
			if (detectCollision(newCoordinates)==true ||findNodeOnList(closedSet, newCoordinates)!=nullptr
                ) {
                continue;
            }
			//寻找到了可以行走的栅格
			//计算代价
            uint totalCost = current->G + 1;//每走一步加一秒 

            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
            	//在开启集中未找到：加入开启集 计算代价
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                /*
                std::cout<<"position:"<<successor->coordinates.x<<","<<\
				successor->coordinates.y<<" G="<<successor->G<<" H="<<successor->H<<" source="<<successor->getScore()<<std::endl;
                */
				openSet.insert(openSet.begin(),successor);
            }
            else if (totalCost <= successor->G) {
            	//开启集中已存在：判断是否代价更小
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }
    CoordinateList path;
	Node *temp=current;
	while (temp != nullptr) {
	path.push_back(temp->coordinates);
	temp = temp->parent;
	}
    
    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

//计算这条路径所花费的时间(s) 
int AStar::Generator::calculateTimeWasted(CoordinateList path)
{
	int time=path.size();
	Vec2i pre,now;
	for(int i=0;i+2<path.size();++i)
	{
		pre.x=path[i+1].x-path[i].x;
		pre.y=path[i+1].y-path[i].y;
		now.x=path[i+2].x-path[i+1].x;
		now.y=path[i+2].y-path[i+1].y;
		if(pre.x!=now.x&&pre.y!=now.y)time+=2;//转弯时间 
	}
	return time;
}

//从nodes_中寻找是否存在coordinates节点 
AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeVec& nodes_,Vec2i coordinates_)
{
	for(auto node:nodes_){
		if(node->coordinates==coordinates_){
			return node;
		}
	}
	return nullptr;
}

//释放节点集 
void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

void AStar::Generator::releaseNodes(NodeVec& nodes_)
{
	for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
    	it = nodes_.erase(it);
    }
}
//检测coordinates_是否超出区域范围或者是否为障碍 
bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

//以下是三种传统的计算启发值的函数 
AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>( delta.x + delta.y);
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));//将函数返回结果处的对象的所有权转移到delta变量 为性能考虑 
    return static_cast<uint>(sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return  (delta.x + delta.y) + std::min(delta.x, delta.y);
}

//打印地图 
void AStar::Generator::printMap()
{
	for(int i=0;i<worldSize.x;i++)
	{
		for(int j=0;j<worldSize.y;j++)
		{
			if(detectCollision({i,j}))
			std::cout<<"■";
			else std::cout<<"□"; 
		}
		std::cout<<std::endl; 
	}
}

//打印路径 
void AStar::Generator::printPath(CoordinateList path)
{
	for(int i=0;i<worldSize.x;i++)
	{
		for(int j=0;j<worldSize.y;j++)
		{
			Vec2i v;
			v.x=i;
			v.y=j;
			if(detectCollision(v))
			std::cout<<"■";
			else
			{
				bool flag=false;//是否找到 
				for(int temp=0;temp<path.size();temp++)
				{
					if(path[temp]==v)
					{
						flag=true;
						break;
					}
				}
				if(flag)std::cout<<"●";
				else std::cout<<"□"; 
			}
			
		}
		std::cout<<std::endl; 
	}
	std::cout<<std::endl; 
}
