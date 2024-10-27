#ifndef ROBOT_ACT_H
#define ROBOT_ACT_H
/*---------------头文件定义区---------------*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include "struct.h"
#include <sstream>
#include <string.h>
#include <stdlib.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "xfyun_waterplus/IATSwitch.h"
#include "wpb_home_tutorials/Follow.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <waterplus_map_tools/Waypoint.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include <wpb_yolo5/BBox2D.h>
#include <wpb_yolo5/BBox3D.h>
#include <sound_play/SoundRequest.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "xfyun_waterplus/IATSwitch.h"
#include <waterplus_map_tools/GetWaypointByName.h>
#include <robot_voice/StringToVoice.h>
#include <sensor_msgs/JointState.h>
#include <thread>
#include <chrono>
/// @brief YOLOV5 BoundingBox2D 格式
typedef struct BBox2D
{
    std::string name;
    int left;
    int right;
    int top;
    int bottom;
    float probability;
} BBox2D;

using namespace cv;
class RobotAct
{
public:
	/*--------------定义区---------------*/
	vector<string> arKWPlacement; // 地点
	vector<string> arKWObject;	  // 物品
	vector<string> arKWPerson;	  // 人名
	vector<string> arKWAction;	  // 行为
	vector<string> strPerson;     // YOLO人识别
	vector<string> objPlacement;  // 垃圾地点
	/*--------------初始化---------------*/
	RobotAct();                   // 构造函数
	~RobotAct();                  // 析构函数
	void Init();                  // 初始化
	/*--------------状态机---------------*/
    vector<stAct> arAct;          // 状态容器
	int nCurActIndex;             // 状态指针
	int nCurActCode;              // 状态码
	bool Main();                  // 主状态机
	void Reset();                 // 状态机重置
	/*--------------回调函数--------------*/
 	void GrabResultCallback(const std_msgs::String::ConstPtr& res);
	void PassResultCallback(const std_msgs::String::ConstPtr& res);
	void YOLOV5Callback(const wpb_yolo5::BBox2D& msg);
	void OpenPoseCallback(const std_msgs::String::ConstPtr& msg);
	void FaceRecogCallback(const std_msgs::String::ConstPtr& msg);
	bool ChatterCallback(robot_voice::StringToVoice::Request &req, robot_voice::StringToVoice::Response &resp);
	/*--------------更新频率--------------*/
	void updateFlagbPeopleFound(); // 人识别标志位更新
	void updateFlagbObjectFound(); // 物识别标志位更新
	void startFlagUpdater();       // 多线程函数
	/*--------------程序功能--------------*/
	void Parameter_Check();        // Yaml参数打印
	void ShowActs();               // 行为队列显示
	string GetToSpeak();           // 说话（废弃）
	string FindWord_Yolo(vector<BBox2D> &YOLO_BBOX, vector<string> &arWord); // 关键字查找 
	string FindWord(string, vector<string> &arWord);                         // 关键字查找
	void   State_Reset();          // 状态重置
	/*--------------机器功能--------------*/
	void  AddNewWaypoint(string);  // 新航点添加
	void  SetSpeed(float,float,float); // 速度设置
	float VelFixed(float,float);   // 速度修正
	bool  Goto(string);            // 航点导航
	void  Enter();                 // 进门
	void  Exit();                  // 出门
	void  GrabSwitch(bool);        // 抓取开关
	void  PassSwitch(bool);        // 递给开关
	void  Speak(const std::string &answer_txt); //机器人说话
	void  Raise_arm();             // 机械臂抬起
	void  Grab_arm();              // 机械臂抓取
	void  Pass_arm();              // 机械臂递给
    /*--------------机器任务--------------*/
	void  ActionDetect();		   // 动作识别
	void  ObjDetect();             // 物品识别
	void  FaceDetect();            // 人脸识别
	/*--------------标志返回--------------*/
	bool  GetFlag_PeopleFound();   // 是否找到人
	bool  GetFlag_ObjectFound();   // 是否找到物
	bool  GetResult_FaceRecog();   // 人脸识别结果
	bool  GetResult_ActionDetect();// 动作识别结果
	bool  GetResult_Grab();        // 抓取结果
	bool  GetResult_Pass();        // 递给结果
	bool  GetResult_FixView();     // 位姿修正结果
	bool  GetResult_bPeopleFoundFailed(); // 人体识别失败
	bool  GetResult_bObjectFoundFailed(); // 物品识别失败
	string getActionFromOpenpose();// 动作识别结果获取
	string getFaceFromFacerecog(); // 人脸识别结果获取
	/*--------------静态变量--------------*/
	static int nPeopleCount;	   // 人物计数
	static int nLitterCount;	   // 垃圾计数
	static int nPlaceCount;	       // 地点计数
	static int nObjPlaceCount;     // 垃圾航点
	bool GlobalbPeopleFound;       // 全局人体识别结果
	bool GlobalbObjectFound;       // 全局物体识别结果
	string coord_dustbin;          // 垃圾桶坐标
	string strListen;			   // 语音识别
	string strDetect;		   	   // YOLO物品识别
	bool bArrive      = false;	   // 到达标志位
	bool bKeyVoice    = false;     // 语音识别开关
	bool _bFixView    = false;     // 位姿修正
	bool _bFixView_ok = false;     // 修正状态
private:
	/*--------------ROS定义区---------------*/
	ros::Publisher  speak_pub;     // 发布者：语音输出
	ros::Publisher  speed_pub;     // 发布者：速度输出
	ros::Publisher  yolo_pub;      // 发布者：YOLO控制
	ros::Publisher  add_waypoint_pub;//发布者：航点添加
	ros::Publisher  behaviors_pub; // 发布者：行为控制
	ros::Subscriber sub_yolo;      // 订阅者：YOLO结果
	ros::Subscriber sub_pose;      // 订阅者：OpenPose结果
	ros::Subscriber sub_face;      // 订阅者：人脸识别结果
	ros::Subscriber grab_result_sub; // 订阅者：抓取结果
	ros::Subscriber pass_result_sub; // 订阅者：递给结果
	ros::ServiceClient client_speak; // 客户端：语音输出
	ros::ServiceClient cliGetWPName; // 客户端：航点名称
	ros::ServiceServer chatter_server_;// 服务端：语音识别
	waterplus_map_tools::GetWaypointByName srvName; // 服务：航点名字
	ros::Publisher mani_ctrl_pub;    // 发布者：机械臂控制
	ros::Publisher ctrl_pub;         // 发布者：底盘控制
	/*---------------类内变量区---------------*/
	int    _check_flag;		   // 程序进入
	string _coord_cmd;  	   // 进门坐标
	string _coord_exit; 	   // 出门坐标
	string _name_yaml;         // 配置文件
	sensor_msgs::JointState mani_ctrl_msg;
	std_msgs::String ctrl_msg;

	int _nActionStage = 1;	   // 动作计数
	int nYoloPeople   = -1;	   // 人物编号

	int _nImgHeight   = 0;	   // 画面中点纵坐标
	int _nImgWidth    = 0;	   // 画面中点横坐标
	int _nTargetX     = 0;	   // 目标人物纵坐标
	int _nTargetY     = 0;	   // 目标人物横坐标

	float _vel_max     = 0.5;  // 移动限速
	float _fVelForward = 0;	   // 修正前进速度
	float _fVelTurn    = 0;	   // 修正转向速度
	float _PID_Forward = 0;	   // 修正前进PID系数
	float _PID_Turn    = 0;	   // 修正转向PID系数

	bool bGrabDone;            // 抓取结果
	bool bPassDone;			   // 递给结果
	bool bOpenpose    = false; // 动作识别
	bool bPeopleFound = false; // 人物标志位
	bool bPeopleFound_failed = false; // 人体识别失败
	bool bObjectFound = false; // 物品标志位
	bool bObjectFound_failed = false; // 物品识别失败
	static bool bActionDetect; // 动作标志位
	bool bFaceDetect  = false; // 人脸标志位
	string GlobalstrAction;    // POSE动作识别
	string strFace;            // FACE人脸识别
	/*---------------数组/容器区---------------*/
	std::vector<BBox2D> YOLO_BBOX;					  // 识别结果
	std::vector<BBox2D>::const_iterator YOLO_BBOX_IT; // 迭代器
	std::vector<BBox2D> recv_BBOX;                    // 识别结果接受
};

#endif 