/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* @author Zhang Wanjie                                             */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <wpv4_behaviors/Coord.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream> //标准输入/输出
#include <wpb_yolo5/BBox3D.h>
#include "SimpleArm.h"

using namespace std;

// 抓取参数调节（单位：米）
static float grab_y_offset = -0.04f;                 //机器人对准物品，横向位移偏移量
static float target_dist = 0.86;                     //伸出手臂抓取前，对准物品的距离
static float grab_z_offset = 0.01;

static float target_x_k = 0.3;                       //对准物品时，前后移动的速度系数
static float target_z_k = 0.3;                       //对准物品时，旋转的速度系数

static float reachout_x = 0.55;                      //伸手的距离
static float forward_count = 3.0;                    //前进计数

static ros::Publisher obj_track_pub;
static geometry_msgs::Pose obj_track_msg;
static ros::Publisher yolo_cmd_pub;
static std_msgs::String yolo_cmd_msg;
static ros::Publisher vel_pub;
static ros::Publisher mani_ctrl_pub;
static sensor_msgs::JointState mani_ctrl_msg;
static ros::Publisher grab_result_pub;
static std_msgs::String grab_result_msg;
static ros::Publisher pt_ctrl_pub;
static sensor_msgs::JointState pt_ctrl_msg;
static CSimpleArm simple_arm;
static bool ManiPoseArrived = false;

static double fDegToAng = 3.1415926/180;
static double fAngToDeg = 180/3.1415926;

static string grab_target_name = "pear";

#define OBJ_POS_NUM 3
typedef struct stObj3dPos
{
    float x;
    float y;
    float z;
}stObj3dPos;
static stObj3dPos obj_3d_pos[OBJ_POS_NUM];

static float obj_track_x = 0.0;
static float obj_track_y = 0.0;
static float obj_track_z = 0.0;

#define STEP_READY             0
#define STEP_INIT_POSE         1
#define STEP_YOLO              2
#define STEP_TAKE_AIM          5
#define STEP_REACH_OUT         6
#define STEP_FORWARD           7
#define STEP_GRAB_OBJ          8
#define STEP_TAKE_OVER         9
#define STEP_BACKWARD          10
#define STEP_DONE              11
int step = STEP_READY;

void VelCmd(float inVx , float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = inVx;
    vel_cmd.linear.y = inVy;
    vel_cmd.angular.z = inTz;
    vel_pub.publish(vel_cmd);
}

void PalmAction(int inAction)
{
    // 0 松开手掌
    if(inAction == 0)
    {
        mani_ctrl_msg.position[6] = 10000; //手爪张开
    }
    // 1 握住手掌
    if(inAction == 1)
    {
        mani_ctrl_msg.position[6] = 45000; //手爪闭合
    }
    mani_ctrl_pub.publish(mani_ctrl_msg);
}

// 目标物体位置连续几帧能稳定识别才能确认
bool Object_Valid(float inX, float inY, float inZ)
{
    // 缓存中所有坐标前移一个位置
    for(int i=0 ;i<OBJ_POS_NUM-1;i++)
    {
        obj_3d_pos[i].x = obj_3d_pos[i+1].x;
        obj_3d_pos[i].y = obj_3d_pos[i+1].y;
        obj_3d_pos[i].z = obj_3d_pos[i+1].z;
    }
    // 新坐标插入最后一个位置
    obj_3d_pos[OBJ_POS_NUM-1].x = inX;
    obj_3d_pos[OBJ_POS_NUM-1].y = inY;
    obj_3d_pos[OBJ_POS_NUM-1].z = inZ;
    // 检测首位坐标差值
    float dist = fabs(obj_3d_pos[0].x - obj_3d_pos[OBJ_POS_NUM-1].x) + fabs(obj_3d_pos[0].y - obj_3d_pos[OBJ_POS_NUM-1].y) + fabs(obj_3d_pos[0].z - obj_3d_pos[OBJ_POS_NUM-1].z);
    if(dist > 0.05)
        return false;
    else
        return true;
}

void Yolo3dCB(const wpb_yolo5::BBox3D::ConstPtr &msg)
{
    // YOLO正在识别
    if(step == STEP_YOLO)
    {
        // 从识别结果中找抓取目标的信息
        int nNumObj = msg->name.size();
        for(int i=0;i<nNumObj;i++)
        {
            if(msg->name[i] == grab_target_name)
            {
                obj_track_x = msg->x_min[i];
                obj_track_y = (msg->y_min[i] + msg->y_max[i])/2;
                obj_track_z = (msg->z_min[i] + msg->z_max[i])/2;
                bool res = Object_Valid(obj_track_x,obj_track_y,obj_track_z);
                if(res == true)
                {
                    ROS_INFO("[Yolo3dCB] 抓取目标 %s  (%.2f , %.2f , %.2f)",msg->name[i].c_str(),obj_track_x,obj_track_y,obj_track_z);

                    // 让YOLO停止识别
                    yolo_cmd_msg.data = "yolo stop";
                    yolo_cmd_pub.publish(yolo_cmd_msg);
                    // 发送到 object_track_node 去锁定
                    obj_track_msg.position.x = obj_track_x;
                    obj_track_msg.position.y = obj_track_y;
                    obj_track_msg.position.z = obj_track_z;
                    obj_track_pub.publish(obj_track_msg);
                    step = STEP_TAKE_AIM;
                }
                break;
            }
        }
    }
}


void ObjTrackCB(const geometry_msgs::Pose::ConstPtr &msg)
{
    if(step == STEP_TAKE_AIM)
    {
        // 获取盒子检测结果
        obj_track_x = msg->position.x;
        obj_track_y = msg->position.y;
        obj_track_z = msg->position.z;
        // ROS_INFO("[ObjTrackCB] 锁定目标 (%.2f , %.2f , %.2f)",obj_track_x,obj_track_y,obj_track_z);
    }
}

void GrabObjCallback(const std_msgs::String::ConstPtr& msg)
{
    if(step != STEP_READY)
        return;
    // 目标物品的坐标
    grab_target_name = msg->data;

    mani_ctrl_msg.position[0] = 0;
    mani_ctrl_msg.position[1] = 70;
    mani_ctrl_msg.position[2] = 90;
    mani_ctrl_msg.position[3] = 120;
    mani_ctrl_msg.position[4] = -90;
    mani_ctrl_msg.position[5] = 80;
    mani_ctrl_msg.position[6] = 35000; //手爪闭合
    PalmAction(0);
    ManiPoseArrived = false;
    step = STEP_INIT_POSE;
}

float VelFixed(float inValue, float inMin, float inMax)
{
    float retValue = inValue;
    if(retValue > inMax)
    {
        retValue = inMax;
    }
    if(retValue < inMin)
    {
        retValue = inMin;
    }
    return retValue;
}


void JointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if(step != STEP_INIT_POSE && step != STEP_REACH_OUT && step != STEP_TAKE_OVER)
        return;
    int nNumJoint = msg->position.size();
    if(nNumJoint != 7)
    {
        //不是机械臂的tf
        return;
    }
    bool bAllPosArrived = true;
    for(int i=0;i<nNumJoint-1;i++)
    {
        float fPosDiff = fabs(mani_ctrl_msg.position[i] - msg->position[i]*fAngToDeg);
        if(fPosDiff > 2.0)
        {
            bAllPosArrived = false;
            break;
        }
    }
    if(bAllPosArrived == true)
    {
        ROS_WARN("[yolo_grab_node] 姿态运动到位！");
        ManiPoseArrived = true;
    }
}


int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "yolo_grab_node");

    ros::NodeHandle nh;
    ros::Publisher behaviors_pub = nh.advertise<std_msgs::String>("/wpv4/behaviors", 10);
    ros::Subscriber yolo_3d_sub = nh.subscribe("/draw_bbox_3d", 10 , Yolo3dCB);
    yolo_cmd_pub = nh.advertise<std_msgs::String>("/yolo_cmd", 10);
    obj_track_pub =  nh.advertise<geometry_msgs::Pose>("/object_track_target", 10);
    ros::Subscriber obj_result_sub = nh.subscribe("/object_track_pose", 10 , ObjTrackCB);
    ros::Subscriber grab_sub = nh.subscribe("/yolo_grab_obj", 10, GrabObjCallback);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    mani_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpm2/joint_ctrl_degree", 30);
    grab_result_pub = nh.advertise<std_msgs::String>("/wpv4/grab_result", 30);
    pt_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/wpv4_pt/joint_ctrl_degree", 10);
    ros::Subscriber joint_states_sub = nh.subscribe("/joint_states",10,&JointStatesCallback);

    ros::NodeHandle nh_param("~");
    nh_param.getParam("grab/target_z_k", target_z_k);
    //ROS_WARN("[grab_obj] target_z_k = %f",target_z_k);

    tf::TransformListener tf_listener; 
    tf::StampedTransform ar_transform;

    ros::Time time = ros::Time(0);
    
    // 手臂角度初始化
    mani_ctrl_msg.name.resize(7);
    mani_ctrl_msg.position.resize(7);
    mani_ctrl_msg.velocity.resize(7);
    for(int i=0;i<7;i++)
    {
        mani_ctrl_msg.position[i] = 0;
        mani_ctrl_msg.velocity[i] = 1000;
    }
    mani_ctrl_msg.position[0] = 0;
    mani_ctrl_msg.position[1] = 70;
    mani_ctrl_msg.position[2] = 90;
    mani_ctrl_msg.position[3] = 120;
    mani_ctrl_msg.position[4] = -90;
    mani_ctrl_msg.position[5] = 80;
    mani_ctrl_msg.position[6] = 35000; //手爪闭合

    //云台控制消息包
    pt_ctrl_msg.name.resize(2);
    pt_ctrl_msg.position.resize(2);
    pt_ctrl_msg.velocity.resize(2);
    //云台角度
    pt_ctrl_msg.position[0] = 0;
    pt_ctrl_msg.position[1] = 0;
    //云台运动速度
    pt_ctrl_msg.velocity[0] = 1000;
    pt_ctrl_msg.velocity[1] = 1000;

    int nCount = 0;
    ros::Rate r(30);
    while(nh.ok())
    {
        // 对准目标盒子
        if(step == STEP_INIT_POSE)
        {
            mani_ctrl_pub.publish(mani_ctrl_msg);
            if(ManiPoseArrived == true)
            {
                for(int i=0 ;i<OBJ_POS_NUM;i++)
                {
                    obj_3d_pos[i].x = 0;
                    obj_3d_pos[i].y = 0;
                    obj_3d_pos[i].z = 0;
                }
                yolo_cmd_msg.data = grab_target_name;
                yolo_cmd_pub.publish(yolo_cmd_msg);
                step = STEP_YOLO;
            }
        }
        // 对准目标盒子
        if(step == STEP_TAKE_AIM)
        {
            float diff_x = obj_track_x - target_dist;
            float diff_y = obj_track_y - grab_y_offset;

            float vx = VelFixed(diff_x * target_x_k, -0.5, 0.5);
            float vz = VelFixed(diff_y * target_z_k, -0.2, 0.2); 

            VelCmd(vx,0,vz);
            ROS_WARN("对准目标 diff_x= %.3f diff_y = %.3f  vz = %.2f",diff_x, diff_y,vz);
            if(fabs(diff_x) < 0.02 && fabs(diff_y) < 0.03)
            {
                VelCmd(0,0,0);
                std_msgs::String behavior_msg;
                behavior_msg.data = "object_track stop";
                behaviors_pub.publish(behavior_msg);
                double x = reachout_x - 0.24;
                double z = obj_track_z - /*0.16*/0.26 - 0.1585 - 0.0825 + grab_z_offset; //base->joint1 , joint1->joint2 , joint2->joint3
                ROS_INFO("规划的机械臂本地坐标为 ( %.2f , %.2f )",x , z);
                bool res = simple_arm.SetTargetPos(x , z);
                if( res == true)
                {
                    ROS_INFO("规划结果 关节角: [0]=%.2f  [1]=%.2f [2]=%.2f",simple_arm.JointAngle[0] ,simple_arm.JointAngle[1] ,simple_arm.JointAngle[2]);
                    mani_ctrl_msg.position[0] = 0;          //根旋转关节
                    mani_ctrl_msg.position[1] = simple_arm.JointAngle[0];
                    mani_ctrl_msg.position[2] = 90;
                    mani_ctrl_msg.position[3] = simple_arm.JointAngle[1];
                    mani_ctrl_msg.position[4] = -90;
                    mani_ctrl_msg.position[5] = simple_arm.JointAngle[2];
                    mani_ctrl_msg.velocity[5] = 2000;
                    PalmAction(0);  //手爪张开
                    nCount = 0;
                    ManiPoseArrived = false;
                    step = STEP_REACH_OUT;
                    ROS_WARN("**step -> STEP_REACH_OUT**");
                    grab_result_msg.data = "Reach out";
                    grab_result_pub.publish(grab_result_msg);
                }
                else
                {
                    ROS_WARN(" 机械臂规划失败！ 请检查坐标数值……");
                    step = STEP_DONE;
                }
            }
        }

        // 伸手准备抓取
        if(step == STEP_REACH_OUT)
        {
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nCount ++;
            if((nCount > 20* 30) || ManiPoseArrived == true)
            {
                nCount = 0;
                step = STEP_FORWARD;
                ROS_WARN("**step -> STEP_FORWARD**");
                grab_result_msg.data = "Forward";
                grab_result_pub.publish(grab_result_msg);
            }
        }

        // 前进进行抓取
        if(step == STEP_FORWARD)
        {
            VelCmd(0.1,0,0);
           // mani_ctrl_msg.position[0] = 0;          //根旋转关节
            //mani_ctrl_pub.publish(mani_ctrl_msg);
            nCount ++;
            // if(nCount > forward_count * 30)
            {
                nCount = 0;
                step = STEP_GRAB_OBJ;
                ROS_WARN("**step -> STEP_GRAB_OBJ**");
                grab_result_msg.data = "Grab";
                grab_result_pub.publish(grab_result_msg);
            }
        }

        // 闭合手爪
        if(step == STEP_GRAB_OBJ)
        {
            VelCmd(0,0,0);
            PalmAction(1);
            nCount ++;
            if(nCount > 6* 30)
            {
                nCount = 0;
                ManiPoseArrived = false;
                mani_ctrl_msg.position[0] = 0;
                mani_ctrl_msg.position[1] = 70;
                mani_ctrl_msg.position[2] = 90;
                mani_ctrl_msg.position[3] = 120;
                mani_ctrl_msg.position[4] = -90;
                mani_ctrl_msg.position[5] = 80;
                mani_ctrl_msg.velocity[5] = 500;
                mani_ctrl_pub.publish(mani_ctrl_msg);
                step = STEP_TAKE_OVER;
                ROS_WARN("**step -> STEP_TAKE_OVER**");
                grab_result_msg.data = "Take over";
                grab_result_pub.publish(grab_result_msg);
            }
        }

        // 收回手臂
        if(step == STEP_TAKE_OVER)
        {
            VelCmd(0,0,0);
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nCount ++;
            if(nCount > 20* 30 || ManiPoseArrived == true)
            {
                nCount = 0;
                step = STEP_BACKWARD;
                ROS_WARN("**step -> STEP_BACKWARD**");
                grab_result_msg.data = "Backward";
                grab_result_pub.publish(grab_result_msg);
            }
        }

        // 后退
        if(step == STEP_BACKWARD)
        {
            VelCmd(-0.1,0,0);
            nCount ++;
            // if(nCount > 3* 30)
            {
                nCount = 0;
                step = STEP_DONE;
                ROS_WARN("**step -> STEP_DONE**");
            }
        }

        // 结束
        if(step == STEP_DONE)
        {
            mani_ctrl_msg.velocity[5] = 1000;
            if(nCount < 5)
            {
                VelCmd(0,0,0);
                grab_result_msg.data = "done";
                grab_result_pub.publish(grab_result_msg);
            }
            nCount ++;
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
