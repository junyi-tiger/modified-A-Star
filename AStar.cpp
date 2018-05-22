#include "AStar.h"
#include <algorithm>
#include<iostream>
using namespace std::placeholders;
//��������+���������
bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

//�ڵ���� ���ڷ���Ŀ���
AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

//Node���캯��
AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

//�������� 
AStar::uint AStar::Node::getScore()
{
    return G + H;
}

//Ĭ�Ϲ��캯�� 
AStar::Generator::Generator()
{
    setHeuristic(&Heuristic::manhattan);
    //���������ѡȡ�����ȵķŵ����� 
    direction1 = {
    	//���յ�λ��λ��ж��λ�õ����ұ�ʱ  type=1
    	//Ѱ·ʱ����ѡȡ����ķ���
    	//���ҡ��ϡ���
    	{ 0, 1 },{ 0, -1 }, { -1, 0 },{ 1, 0 },
    };
    direction2 = {
    	// ���յ�λ��λ��ж��λ�õ��ϡ��±�ʱ type=0
    	//Ѱ·ʱ����ѡȡ����ķ���
    	 //�ϡ��¡����� 
		{ -1, 0 }, { 1, 0 }, { 0 , -1 }, { 0, 1 },
	};
}

//���õ�ͼ��С 
void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

//����4�����˶����ǰ˷��� 

//���������������� 
void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

//����ϰ� 
void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

//�Ƴ��ϰ� 
void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

//����ϰ� 
void AStar::Generator::clearCollisions()
{
    walls.clear();
}

//Ѱ��һ����source_��target_��·��������Ѱ�ҵ���·����դ���б� 
AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_,int type)//type�����յ����� 
{
	//A star�㷨����
    Node *current = nullptr;
    NodeVec openSet; 
    NodeSet closedSet;
	openSet.insert(openSet.begin(),new Node(source_));
	CoordinateList ans;
	int minTimeWasted=INT_MAX;
	//����������Ϊ��ʱ 
    while (!openSet.empty()) {
        current = openSet[0];//��ÿ������ĵ�һ��
        for (auto node : openSet) {//�����С���۵Ľڵ㣬��ֵΪcurrent 
            if (node->getScore() <= current->getScore()) {
                current = node;
            }
        }

        if (current->coordinates == target_) {//����Ŀ�ĵ�
        	break; 
        	/*
        	//������ʱ�����
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
			openSet.erase(std::find(openSet.begin(), openSet.end(), current));//��openSet��ɾ����current�ڵ� 
        	if(openSet.empty())//Ϊ�յĻ���������Ѱ 
            break;
            //��Ϊ�գ�������Ѱ 
			continue; 
			*/
        }

        closedSet.insert(current);//��current���뵽�رռ�
        openSet.erase(std::find(openSet.begin(), openSet.end(), current));//��current�ӿ�������ɾ��

        for (uint i = 0; i < 4; ++i) {
        	Vec2i newCoordinates;
        	if(type)//�յ���ж��λ�õ����ұ�
            newCoordinates=current->coordinates + direction1[i];//��˳�����ĸ�������ĳ�������ϵ�դ��λ��
            else newCoordinates=current->coordinates + direction2[i];
			if (detectCollision(newCoordinates)==true ||findNodeOnList(closedSet, newCoordinates)!=nullptr
                ) {
                continue;
            }
			//Ѱ�ҵ��˿������ߵ�դ��
			//�������
            uint totalCost = current->G + 1;//ÿ��һ����һ�� 

            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
            	//�ڿ�������δ�ҵ������뿪���� �������
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
            	//���������Ѵ��ڣ��ж��Ƿ���۸�С
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

//��������·�������ѵ�ʱ��(s) 
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
		if(pre.x!=now.x&&pre.y!=now.y)time+=2;//ת��ʱ�� 
	}
	return time;
}

//��nodes_��Ѱ���Ƿ����coordinates�ڵ� 
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

//�ͷŽڵ㼯 
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
//���coordinates_�Ƿ񳬳�����Χ�����Ƿ�Ϊ�ϰ� 
bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

//���������ִ�ͳ�ļ�������ֵ�ĺ��� 
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
    auto delta = std::move(getDelta(source_, target_));//���������ؽ�����Ķ��������Ȩת�Ƶ�delta���� Ϊ���ܿ��� 
    return static_cast<uint>(sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return  (delta.x + delta.y) + std::min(delta.x, delta.y);
}

//��ӡ��ͼ 
void AStar::Generator::printMap()
{
	for(int i=0;i<worldSize.x;i++)
	{
		for(int j=0;j<worldSize.y;j++)
		{
			if(detectCollision({i,j}))
			std::cout<<"��";
			else std::cout<<"��"; 
		}
		std::cout<<std::endl; 
	}
}

//��ӡ·�� 
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
			std::cout<<"��";
			else
			{
				bool flag=false;//�Ƿ��ҵ� 
				for(int temp=0;temp<path.size();temp++)
				{
					if(path[temp]==v)
					{
						flag=true;
						break;
					}
				}
				if(flag)std::cout<<"��";
				else std::cout<<"��"; 
			}
			
		}
		std::cout<<std::endl; 
	}
	std::cout<<std::endl; 
}
