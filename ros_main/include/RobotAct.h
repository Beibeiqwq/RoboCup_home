#ifndef ROBOT_ACT_H  
#define ROBOT_ACT_H
/*---------------头文件定义区---------------*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <list>
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

typedef struct BBox3D
{
	std::string name;
	std::string frame_id;
	float x_min;
	float x_max;
	float y_min;
	float y_max;
	float z_min;
	float z_max;
} BBox3D;


using namespace cv;
class RobotAct
{
public:
	RobotAct();
	~RobotAct();
	/*--------------数据类---------------*/
	vector<string> arKWPlacement; // 地点
	vector<string> arKWObject;	  // 物品
	vector<string> arKWPerson;	  // 人名
	vector<string> arKWAction;	  // 行为
	/*--------------功能类---------------*/
    std::list<stAct> arAct; 
	int nCurActIndex;
	int nCurActCode;
	void Init();
	void Reset();
	bool Main();
	void ShowActs();
	void Parameter_Check();
	/*--------------导航类---------------*/
	bool Goto(string);
	void Enter();
	void Exit();
	void AddNewWaypoint(string);
	void AddNewWaypoint(tfScalar tx, tfScalar ty);
	/*--------------速度控制--------------*/
	float VelFixed(float,float);
	void SetSpeed(float,float,float);
    /*--------------图像类---------------*/
	void YoloStart();
	void YOLOV5CB(const wpb_yolo5::BBox2D& msg);
	void YOLOV5CB_3D(const wpb_yolo5::BBox3D& msg);
	void OpenPoseCB(const std_msgs::String::ConstPtr& msg);
	void ActionDetect();
	void ProcColorCB(const sensor_msgs::ImageConstPtr& msg);
	string strDetect;		   // 识别到的物体或人名
	/*--------------语音类---------------*/
	std::string strListen;
	void Speak(const std::string &answer_txt);
	string GetToSpeak();
	string FindWord(string, vector<string> &arWord);
	string FindWord_Yolo(vector<BBox2D> &YOLO_BBOX, vector<string> &arWord);
	/*--------------动作类---------------*/
	bool bGrabDone;
	bool bPassDone;
	bool _bFixView = false;	   // 位姿修正
	bool _bFixView_ok = false; // 修正状态
	ros::Subscriber grab_result_sub;
	ros::Subscriber pass_result_sub;
	void GrabSwitch(bool);
	void PassSwitch(bool);
 	void GrabResultCallback(const std_msgs::String::ConstPtr& res);
	void PassResultCallback(const std_msgs::String::ConstPtr& res);
	/*--------------任务类---------------*/
	int nPeopleCount = 0;	   // 人物计数
	int nLitterCount = 0;	   // 垃圾计数
	int nPlaceCount  = 1;	   // 地点计数
	bool bPeopleFound = false; // 人物标志位
	bool bObjectFound = false; // 物品标志位
private:
	/*--------------ROS定义区---------------*/
	ros::Publisher speak_pub;
	ros::Publisher speed_pub;
	ros::Publisher yolo_pub;
	ros::Publisher add_waypoint_pub;
	ros::Publisher behaviors_pub;
	ros::Subscriber sub_yolo;
	ros::Subscriber sub_ent;
	ros::Subscriber sub_pose;
	ros::ServiceClient client_speak;
	ros::ServiceClient cliGetWPName;
	waterplus_map_tools::GetWaypointByName srvName;
	/*---------------类内变量区---------------*/
	int    _check_flag;		   // 程序进入
	string _coord_cmd;  	   //进门坐标
	string _coord_exit; 	   //出门坐标
	string _name_yaml;         //配置文件

	int _nActionStage = 0;	   // 动作计数

	int nYoloPeople = -1;	   // 人物编号

	int _nImgHeight = 0;	   // 画面中点纵坐标
	int _nImgWidth = 0;		   // 画面中点横坐标
	int _nTargetX = 0;		   // 目标人物纵坐标
	int _nTargetY = 0;		   // 目标人物横坐标

	float _vel_max = 0.5;	   // 移动限速
	float _fVelForward = 0;	   // 修正前进速度
	float _fVelTurn = 0;	   // 修正转向速度
	float _PID_Forward = 0;	   // 修正前进PID系数
	float _PID_Turn = 0;	   // 修正转向PID系数

	//bool bArrive = false;	   // 到达标志位

	bool bOpenpose = false;	   // 动作识别
	/*---------------数组/容器区---------------*/
	std::vector<BBox2D> YOLO_BBOX;					  // 识别结果
	std::vector<BBox3D> YOLO_BBOX_3D;
	std::vector<BBox2D>::const_iterator YOLO_BBOX_IT; // 迭代器
	std::vector<BBox2D> recv_BBOX;
	std::vector<BBox3D> recv_BBOX_3D;
	std::list<stAct>::iterator ARACT_IT = arAct.begin();

};

#endif 