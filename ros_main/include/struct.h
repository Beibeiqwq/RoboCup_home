#ifndef STRUCT_H
#define STRUCT_H
#include<string>
using namespace std;
#define ACT_REMOVE          0
#define ACT_GOTO			1
#define ACT_GRAB			2
#define ACT_PASS			3
#define ACT_SPEAK			4
#define ACT_LISTEN			5
#define ACT_MOVE			6
#define ACT_ADD_WAYPOINT	7
#define ACT_FIND_OBJ        8
#define ACT_FIND_PERSON     9

typedef struct stAct
{
	int    nAct;			//行为号
	string strTarget;	    //移动目标航点名称/语音说话内容/语音识别关键词
	float  nDuration;	    //语音持续时间
	float  fLinear_x;		//前后平移移动
	float  fLinear_y;		//左右平移速度
	float  fAngular_z;	    //旋转速度,正值向左旋转,负值向右旋转
}stAct;


#endif