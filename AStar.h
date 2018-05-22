/*
	author by YiJun. 18/5/19
	email:yijun0226@foxmail.com
*/
#ifndef __ASTAR__
#define __ASTAR__

#include <vector>
#include <functional>
#include <set>

namespace AStar
{
	//ÿ��դ������� 
    struct Vec2i
    {
        int x, y;

        bool operator == (const Vec2i& coordinates_);
    };
	
	
    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;
    
	struct Node
    {
        uint G, H;
        Vec2i coordinates;
        Node *parent;

        Node(Vec2i coord_, Node *parent_ = nullptr);
        uint getScore();
    };

    using NodeSet = std::set<Node*>;//�ڵ㼯�� 
	using NodeVec = std::vector<Node*>;//�ڵ㼯������ 
	
    class Generator
    {
        bool detectCollision(Vec2i coordinates_);//�жϴ�λ�ô����ϰ� 
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);//�ҽڵ� 
        Node* findNodeOnList(NodeVec& nodes_,Vec2i coordinates_);
        void releaseNodes(NodeSet& nodes_);//�ͷŽڵ� 
        void releaseNodes(NodeVec& nodes_);
        
    public:
        Generator();
        void setWorldSize(Vec2i worldSize_);//���ô�С 
        void setHeuristic(HeuristicFunction heuristic_);//���������������� 
    	CoordinateList findPath(Vec2i source_, Vec2i target_,int type);//Ѱ��һ����source��target��·�� 
        void addCollision(Vec2i coordinates_);//��һ��λ������ϰ� 
        void removeCollision(Vec2i coordinates_);//�Ƴ�ĳ��λ�ô����ϰ� 
        void clearCollisions();//�Ƴ�ȫ���ϰ� 
		
		void printMap();//��ӡ���õĵ�ͼ 
		void printPath(CoordinateList path);//��ӡ����·�� 
		static int calculateTimeWasted(CoordinateList path);//��������·���ϻ��ѵ�ʱ�� 
		
    private:
        HeuristicFunction heuristic;//���� 
        CoordinateList direction1, direction2, walls;//�����Լ��ϰ� 
        Vec2i worldSize;//��ͼ��С
    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static uint manhattan(Vec2i source_, Vec2i target_);
        static uint euclidean(Vec2i source_, Vec2i target_);
        static uint octagonal(Vec2i source_, Vec2i target_);
    };
}

#endif // __ASTAR__
