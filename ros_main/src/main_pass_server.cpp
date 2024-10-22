#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#define STEP_WAIT        0
#define STEP_HAND_UP     1
#define STEP_GRIPPER     2
#define STEP_HAND_DOWN   3
#define STEP_DONE        4

static int nStep = STEP_WAIT;
static ros::Publisher result_pub;
static std_msgs::String result_msg;
static int nDelayCount = 0;

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("pass start");
    if( nFindIndex >= 0 )
    {
        nDelayCount = 0;
        nStep = STEP_HAND_UP;
        ROS_WARN("[pass_start] ");
    }

    nFindIndex = msg->data.find("pass stop");
    if( nFindIndex >= 0 )
    {
        nStep = STEP_WAIT;
        ROS_WARN("[pass_stop] ");
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main_pass_server");

    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/wpb_home/behaviors", 30, BehaviorCB);
    ros::Publisher mani_ctrl_pub = n.advertise<sensor_msgs::JointState>("wpb_home/mani_ctrl", 30);
    result_pub = n.advertise<std_msgs::String>("/wpb_home/pass_result", 30);

    bool bActive = false;
    ros::NodeHandle nh_param("~");
    nh_param.param<bool>("start", bActive, false);
    if(bActive == true)
    {
        nDelayCount = 0;
        nStep = STEP_HAND_UP;
    }

    sensor_msgs::JointState mani_ctrl_msg;
    mani_ctrl_msg.name.resize(2);
    mani_ctrl_msg.position.resize(2);
    mani_ctrl_msg.velocity.resize(2);
    mani_ctrl_msg.name[0] = "lift";
    mani_ctrl_msg.name[1] = "gripper";
    mani_ctrl_msg.position[0] = 0;
    mani_ctrl_msg.velocity[0] = 0.5;     //升降速度(单位:米/秒)
    mani_ctrl_msg.position[1] = 0.16;
    mani_ctrl_msg.velocity[1] = 5;       //手爪开合角速度(单位:度/秒)

    ros::Rate r(10);
    ros::spinOnce();
    r.sleep();
    
    while(ros::ok())
    {
        switch(nStep)
        {
        case STEP_HAND_UP:
            if(nDelayCount == 0)
            {
                result_msg.data = "hand up";
                result_pub.publish(result_msg);
                ROS_WARN("[wpb_home_pass_server] STEP_HAND_UP");
            }
            mani_ctrl_msg.position[0] = 1.0;
            mani_ctrl_msg.position[1] = -0.1;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nDelayCount ++;
            if(nDelayCount > 80)
            {
                nDelayCount = 0;
                nStep = STEP_GRIPPER;
                result_msg.data = "gripper";
                result_pub.publish(result_msg);
                ROS_WARN("[wpb_home_pass_server] STEP_GRIPPER");
            }
            break;
        case STEP_GRIPPER:
            mani_ctrl_msg.position[1] = 0.16;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nDelayCount ++;
            if(nDelayCount > 50)
            {
                nDelayCount = 0;
                nStep = STEP_HAND_DOWN;
                result_msg.data = "hand down";
                result_pub.publish(result_msg);
                ROS_WARN("[wpb_home_pass_server] STEP_HAND_DOWN");
            }
            break;
        case STEP_HAND_DOWN:
            mani_ctrl_msg.position[0] = 0;
            mani_ctrl_msg.position[1] = 0.16;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nDelayCount ++;
            if(nDelayCount > 80)
            {
                nDelayCount = 0;
                nStep = STEP_DONE;
                result_msg.data = "done";
                result_pub.publish(result_msg);
                ROS_WARN("[wpb_home_pass_server] STEP_DONE");
            }
            break;
        case STEP_DONE:
            break;
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}