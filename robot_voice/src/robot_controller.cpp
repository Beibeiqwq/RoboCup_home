#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <robot_voice/StringToVoice.h>
#include <std_msgs/String.h>
#include <vector>
#include <sound_play/SoundRequest.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "xfyun_waterplus/IATSwitch.h"
#include <robot_voice/StringToVoice.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;
class RobotController {
public:
  RobotController() {
    ROS_INFO("RobotController Constructor");
  }

  ~RobotController() {
    ROS_INFO("RobotController Destructor");
  }

  int Init(ros::NodeHandle& nh) {
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    client_ = nh.serviceClient<robot_voice::StringToVoice>("str2voice");
    return 0;
  }

  void ToDownstream(const std::string& answer_txt) {
  	// 通过 str2voice 服务和 /cmd_vel topic向下游 voice_creator 和 mbot_gazebo 发送
    robot_voice::StringToVoice::Request req;
    robot_voice::StringToVoice::Response resp;
    req.data = answer_txt;

    bool ok = client_.call(req, resp);
    if (ok) {
      printf("send str2voice service success: %s, and pub cmd_vel\n", req.data.c_str());
    } else {
      ROS_ERROR("failed to send str2voice service");
    }
  }

  bool ChatterCallbback(robot_voice::StringToVoice::Request &req, robot_voice::StringToVoice::Response &resp) {
    printf("i received: %s\n", req.data.c_str());
    std::string voice_txt = req.data;
	// 根据指令关键字，发送对应的语音播包文字和 cmd_vel 命令
    if (voice_txt.find("你好") != std::string::npos) 
    {
      ToDownstream("你好 我是天天开心小队制作的服务机器人");
    } 
    else if (voice_txt.find("功能") != std::string::npos) 
    {
      ToDownstream("我可以进行人体日常行为识别 还可以自主识别垃圾 并完成房间中的垃圾清理");
    }
    else if(voice_txt.find("请带我去卧室") != std::string::npos)
    {
      ToDownstream("现在开始前往指定地点 卧室");
      sleep(5);
      //Goto("cmd");
    } 
    resp.success = true;
    return resp.success;
  }

  void Start(ros::NodeHandle& nh) {
  	// 申明 human_chatter 服务，ChatterCallbback是回调函数
    chatter_server_ = nh.advertiseService("human_chatter", &RobotController::ChatterCallbback, this);
  }

private:
  ros::ServiceServer chatter_server_;
  ros::Publisher cmd_pub_;
  ros::ServiceClient client_;
  
};

int main(int argc, char* argv[]) {
  int ret = 0;
  ros::init(argc, argv, "voice_controller");
  ros::NodeHandle nh;

  RobotController rc;
  rc.Init(nh);

  rc.Start(nh);

  ros::spin();
  return 0;
}

