/*
	author by YiJun. 18/5/19
	email:yijun0226@foxmail.com
*/
#ifndef __USE_OF_ASTAR__
#define  __USE_OF_ASTAR__

#include "AStar.h"

using namespace AStar;
using namespace std;
namespace Use_AStar
{
	//机器车类
	class Car{
	public:
		int num;
		double time=0.0;//当前时间 
		std::vector<CoordinateList> paths;
		std::vector<double> Times;//走完对应路径的结束时间
		void addPath(Vec2i target_,CoordinateList path,Generator& generator);
	};
    //对地图生成器进行一系列初始化操作 
	void Init(Generator& generator);
	//寻找路径 
	CoordinateList findthePath(Vec2i source_,Vec2i target_,Generator& generator);
	//获得所花费的时间
	int getWaistedTime(CoordinateList path,Generator& generator);
	//开始主步骤的运行
	vector<Car>& run(Generator& generator);
	//读取文件
	std::vector<int>& readFile();
	std::vector<Vec2i>& getTargets(vector<int> node);
}

#endif
