// A门
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sound_play/SoundRequest.h>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "xfyun_waterplus/IATSwitch.h"
#include <waterplus_map_tools/GetWaypointByName.h>
#include <wpb_home_behaviors/Rect.h>
#include <wpb_home_behaviors/Coord.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/Image.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <wpb_home_behaviors/Coord.h>
#include <opencv2/highgui/highgui_c.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
// 有限状态机
#define STATE_READY 0
#define STATE_WAIT_ENTR 1
#define STATE_GOTO_RECO 2
#define STATE_WAIT_RECO 3
#define STATE_CONFIRM 4
#define STATE_GOTO_EXIT 5
#define STATE_WAIT_OBJ 6

// 工作模式
#define MODE_IDLE 0
#define MODE_OBJECT_DETECT 1

// sleep(1000);
using namespace std;

static ros::Publisher spk_pub;
static ros::ServiceClient clientIAT;
static xfyun_waterplus::IATSwitch srvIAT;
static ros::ServiceClient cliGetWPName;
static waterplus_map_tools::GetWaypointByName srvName;
static int nPersonCount = 0;
static std::string pc_topic;
static ros::Publisher pc_pub;
static ros::Publisher marker_pub;
static ros::Publisher coord_pub;
static tf::TransformListener *tf_listener;
static visualization_msgs::Marker line_box;
static visualization_msgs::Marker line_follow;
static visualization_msgs::Marker text_marker;
static ros::Publisher segmented_objects;
static ros::Publisher segmented_plane;
static int nMode = MODE_IDLE;
static int nState = STATE_WAIT_ENTR; // 程序启动时初始状态
static int State = 0, coun = 1, facenum, obj = 1;

// 识别关键词
static vector<string> arKWPerson;
static vector<string> arKWConfirm;
static vector<string> arKWObject;

float ynum[3] = {10.0, 10.0, 10.0};
int facex[3] = {0, 0, 0};

void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB);
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB);
void DrawPath(float inX, float inY, float inZ);
void RemoveBoxes();
void SortObjects();

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void FaceRectCB(const wpb_home_behaviors::Rect::ConstPtr &msg)
{
    int nNumFace = msg->name.size();
    facenum = nNumFace;
    ROS_WARN("[FaceRectCB] num = %d", nNumFace);
    if (nNumFace > 0)
    {
        for (int i = 0; i < nNumFace; i++)
        {
            ROS_WARN("[FaceRectCB] %s -(%d , %d , %d , %d)", msg->name[i].c_str(), msg->top[i], msg->bottom[i], msg->left[i], msg->right[i]);
            facex[i] = (msg->left[i] + msg->right[i]) / 2;
        }
    }
}

void FaceCoordCB(const wpb_home_behaviors::Coord::ConstPtr &msg)
{
    int nNumFace = msg->name.size();
    facenum = nNumFace;
    ROS_WARN("[FaceCoordCB] num = %d", nNumFace);
    if (nNumFace > 0)
    {
        for (int i = 0; i < nNumFace; i++)
        {
            ROS_WARN("[FaceCoordCB] %s -(%.2f , %.2f , %.2f)", msg->name[i].c_str(), msg->x[i], msg->y[i], msg->z[i]);
            ynum[i] = msg->y[i];
        }
    }
}

static void Init_keywords()
{
    // 人名关键词(根据比赛前一天提供的人名列表进行修改)
    arKWPerson.push_back("Jack");
    arKWPerson.push_back("Linda");
    arKWPerson.push_back("Lucy");
    arKWPerson.push_back("Grace");
    arKWPerson.push_back("John");
    arKWPerson.push_back("Allen");
    arKWPerson.push_back("Richard");
    arKWPerson.push_back("Mike");
    arKWPerson.push_back("Lily");

    // yes or no
    arKWConfirm.push_back("yes");
    arKWConfirm.push_back("Yes");
    arKWConfirm.push_back("Yeah");

    // 物品
    arKWObject.push_back("Water");
    arKWObject.push_back("Chip");
    // arKWObject.push_back("Cheap");
    arKWObject.push_back("Sprit");
    arKWObject.push_back("Cola");
    arKWObject.push_back("Biscuit");
    arKWObject.push_back("Bread");
    arKWObject.push_back("Lays");
    arKWObject.push_back("Cookie");
    arKWObject.push_back("Hand wash");
    arKWObject.push_back("Orange juice");
    arKWObject.push_back("Dish soap");
}

static string FindWord(string inSentence, vector<string> &arWord)
{
    string strRes = "";
    int nNum = arWord.size();
    for (int i = 0; i < nNum; i++)
    {
        int tmpIndex = inSentence.find(arWord[i]);
        if (tmpIndex >= 0)
        {
            strRes = arWord[i];
            break;
        }
    }
    return strRes;
}

static void Speak(string inStr)
{
    sound_play::SoundRequest sp;
    sp.sound = sound_play::SoundRequest::SAY;
    sp.command = sound_play::SoundRequest::PLAY_ONCE;
    sp.arg = inStr;
    sp.volume = 1.0f; // indigo(Ubuntu 14.04)需要注释掉这一句才能编译
    spk_pub.publish(sp);
}

static bool Goto(string inStr)
{
    string strGoto = inStr;
    srvName.request.name = strGoto;
    if (cliGetWPName.call(srvName))
    {
        std::string name = srvName.response.name;
        float x = srvName.response.pose.position.x;
        float y = srvName.response.pose.position.y;
        ROS_INFO("Get_wp_name: name = %s (%.2f,%.2f)", strGoto.c_str(), x, y);

        MoveBaseClient ac("move_base", true);
        if (!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("The move_base action server is no running. action abort...");
            return false;
        }
        else
        {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = srvName.response.pose;
            ac.sendGoal(goal);
            ac.waitForResult();
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Arrived at %s!", strGoto.c_str());
                return true;
            }
            else
            {
                ROS_INFO("Failed to get to %s ...", strGoto.c_str());
                return false;
            }
        }
    }
    else
    {
        ROS_ERROR("Failed to call service GetWaypointByName");
        return false;
    }
}

static int nOpenCount = 0;
void EntranceCB(const std_msgs::String::ConstPtr &msg)
{
    // ROS_WARN("[WhoIsWho EntranceCB] - %s",msg->data.c_str());
    string strDoor = msg->data;
    if (strDoor == "door open")
    {
        nOpenCount++;
    }
    else
    {
        nOpenCount = 0;
    }
}

static bool bGotoExit = false;
void KeywordCB(const std_msgs::String::ConstPtr &msg)
{
    ROS_WARN("[WhoIsWho KeywordCB] - %s", msg->data.c_str());
    string strListen = msg->data;

    if (nState == STATE_WAIT_RECO)
    {
        // 从听到的句子里找人名
        string person = FindWord(strListen, arKWPerson);
        if (person.length() > 0)
        {
            nState = STATE_CONFIRM;

            printf("句子里包含人名 - %s \n", person.c_str());
            string strRepeat = "your name is " + person;
            Speak(strRepeat);
        }
    }

    if (nState == STATE_WAIT_OBJ)
    {
        // 从听到的句子里找物品
        string object = FindWord(strListen, arKWObject);
        if (object.length() > 0)
        {
            nState = STATE_CONFIRM;

            printf("句子里包含物品 - %s \n", object.c_str());
            string strRepeat = "your object is " + object;
            Speak(strRepeat);
        }
    }

    if (nState == STATE_CONFIRM)
    {
        string confirm = FindWord(strListen, arKWConfirm);
        if (confirm == "yes" || confirm == "Yes" || confirm == "Yeah")
        {
            if (obj == 0)
            {
                obj++;
                Speak("ok,I have memory your object.");
                nState = STATE_WAIT_RECO;
                State++;
                nPersonCount++;
                ynum[0] = ynum[1] = ynum[2] = 10.0;
                facex[0] = facex[1] = facex[2] = 0;
            }
            else if (obj == 1)
            {
                Speak("ok,I have memory you.What do you want");
                nState = STATE_WAIT_OBJ;
                obj = 0;
                ynum[0] = ynum[1] = ynum[2] = 10.0;
                facex[0] = facex[1] = facex[2] = 0;
            }
            if (nPersonCount >= 4) // 要识别的人名个数
            {
                bGotoExit = true;
                nState = STATE_GOTO_EXIT;
            }
        }
    }
}

void RestrictROI()
{
    while (true)
    {
        ynum[0] = ynum[1] = ynum[2] = 10.0;
        ros::spinOnce();
        if ((ynum[0] > -0.1 && ynum[0] < 0.2) || (ynum[1] > -0.1 && ynum[1] < 0.2) || (ynum[2] > -0.1 && ynum[2] < 0.2))
        {
            if ((facex[0] > 730 && facex[0] < 1080) || (facex[1] > 730 && facex[1] < 1080) || (facex[2] > 730 && facex[2] < 1080))
            {
                ynum[0] = ynum[1] = ynum[2] = 10.0;
                break;
            }
        }
    }
}

geometry_msgs::Twist Pub_vel(float linear_x, float linear_y, float linear_z, float angular_x, float angular_y, float angular_z)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = linear_x;
    vel_cmd.linear.y = linear_y;
    vel_cmd.linear.z = linear_z;
    vel_cmd.angular.x = angular_x;
    vel_cmd.angular.y = angular_y;
    vel_cmd.angular.z = angular_z;
    return vel_cmd;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "study1");
    Init_keywords();

    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/xfyun/iat", 10, KeywordCB);
    ros::Subscriber sub_ent = n.subscribe("/wpb_home/entrance_detect", 10, EntranceCB);
    ros::Subscriber face_2d_sub = n.subscribe("/wpb_home/face_detect_2d", 1, FaceRectCB);
    ros::Subscriber face_3d_sub = n.subscribe("/wpb_home/face_detect_3d", 1, FaceCoordCB);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    clientIAT = n.serviceClient<xfyun_waterplus::IATSwitch>("xfyun_waterplus/IATSwitch");
    cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    spk_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20);

    ROS_INFO("[main] wpb_home_WhoIsWho");
    ros::Rate r(10);
    while (ros::ok())
    {
        if (nState == STATE_WAIT_ENTR)
        {
            // 等待开门,一旦检测到开门,便去往发令地点
            if (nOpenCount > 20) // 前方物体与机器距离大于20
            {
                Goto("cmd");
                Speak("I have reached the target position");
                sleep(2);
                bool bArrived = Goto("dining room");
                // 寻找人体（空）

                sleep(2);
                if (bArrived == true)
                {
                    vel_pub.publish(Pub_vel(0, 0, 0, 0, 0, 0.03));
                    RestrictROI();
                    vel_pub.publish(Pub_vel(0, 0, 0, 0, 0, 0));
                    Speak("Tell me your name");
                    nState = STATE_WAIT_RECO;
                    srvIAT.request.active = true;
                    srvIAT.request.duration = 3;
                    clientIAT.call(srvIAT);
                    // bArrived = false;
                }
            }
        }
        if (nState == STATE_WAIT_RECO && State == 1 && coun == 1)
        {

            vel_pub.publish(Pub_vel(0, 0, 0, 0, 0, 0.03));
            sleep(5.4);
            RestrictROI();
            vel_pub.publish(Pub_vel(0, 0, 0, 0, 0, 0));
            coun = 0;
            Speak("Tell me your name");
            nState = STATE_WAIT_RECO;
            srvIAT.request.active = true;
            srvIAT.request.duration = 3;
            clientIAT.call(srvIAT);
            // bArrived = false;
        }
        if (nState == STATE_WAIT_RECO && State == 2 && coun == 0)
        {
            geometry_msgs::Twist vel_cmd;
            sleep(7);
            vel_pub.publish(Pub_vel(0, 0, 0, 0, 0, 0.03));
            sleep(5.4);
            RestrictROI();
            vel_pub.publish(Pub_vel(0, 0, 0, 0, 0, 0));
            coun = 1;
            Speak("Tell me your name");
            nState = STATE_WAIT_RECO;
            srvIAT.request.active = true;
            srvIAT.request.duration = 3;
            clientIAT.call(srvIAT);

            // bArrived = false;
        }
        if (nState == STATE_GOTO_EXIT && bGotoExit == true)
        {
            bGotoExit = false;

            // 识别完毕,关闭语音识别
            srvIAT.request.active = false;
            clientIAT.call(srvIAT);

            Speak("ok,I have memory you. I am leaving.");
            Goto("exit");
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
