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
	//每个栅格的坐标 
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

    using NodeSet = std::set<Node*>;//节点集合 
	using NodeVec = std::vector<Node*>;//节点集合向量 
	
    class Generator
    {
        bool detectCollision(Vec2i coordinates_);//判断此位置处的障碍 
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);//找节点 
        Node* findNodeOnList(NodeVec& nodes_,Vec2i coordinates_);
        void releaseNodes(NodeSet& nodes_);//释放节点 
        void releaseNodes(NodeVec& nodes_);
        
    public:
        Generator();
        void setWorldSize(Vec2i worldSize_);//设置大小 
        void setHeuristic(HeuristicFunction heuristic_);//设置启发函数类型 
    	CoordinateList findPath(Vec2i source_, Vec2i target_,int type);//寻找一条从source到target的路径 
        void addCollision(Vec2i coordinates_);//在一个位置添加障碍 
        void removeCollision(Vec2i coordinates_);//移除某个位置处的障碍 
        void clearCollisions();//移除全部障碍 
		
		void printMap();//打印建好的地图 
		void printPath(CoordinateList path);//打印这条路径 
		static int calculateTimeWasted(CoordinateList path);//计算这条路径上花费的时间 
		
    private:
        HeuristicFunction heuristic;//类型 
        CoordinateList direction1, direction2, walls;//方向以及障碍 
        Vec2i worldSize;//地图大小
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
