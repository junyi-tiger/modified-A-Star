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
	//��������
	class Car{
	public:
		int num;
		double time=0.0;//��ǰʱ�� 
		std::vector<CoordinateList> paths;
		std::vector<double> Times;//�����Ӧ·���Ľ���ʱ��
		void addPath(Vec2i target_,CoordinateList path,Generator& generator);
	};
    //�Ե�ͼ����������һϵ�г�ʼ������ 
	void Init(Generator& generator);
	//Ѱ��·�� 
	CoordinateList findthePath(Vec2i source_,Vec2i target_,Generator& generator);
	//��������ѵ�ʱ��
	int getWaistedTime(CoordinateList path,Generator& generator);
	//��ʼ�����������
	vector<Car>& run(Generator& generator);
	//��ȡ�ļ�
	std::vector<int>& readFile();
	std::vector<Vec2i>& getTargets(vector<int> node);
}

#endif
