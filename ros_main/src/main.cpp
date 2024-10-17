#include <ros/ros.h>
#include <RobotAct.h> 
/*---------------状态机区---------------*/
#define STATE_READY               0
#define STATE_WAIT_ENTR           1
#define STATE_WAIT_CMD            2
#define STATE_ACTION              3
#define STATE_GOTO_EXIT           4

#define TimerAct_READY            5
#define TimerAct_FIND_PERSON      6
#define TimerAct_FIND_OBJ         7
/*---------------初始化区---------------*/


static RobotAct Robot;
/*--------------ROS定义区---------------*/
ros::Timer Task_Timer;
/*---------------全局变量区---------------*/
static int TimerAct = TimerAct_READY;// 任务状态
static int nState   = STATE_READY;   // 初始状态
static int nOpenCount = 0;           // 开门延迟
static string strDetect;             // 物品识别
/*---------------数组/容器区---------------*/
std::vector<BBox2D> YOLO_BBOX;                    // 识别结果
std::vector<BBox2D>::const_iterator YOLO_BBOX_IT; // 迭代器
std::vector<BBox2D> recv_BBOX;
std::vector<string> arKWPlacement;   // 地点
std::vector<string> arKWObject;      // 物品
std::vector<string> arKWPerson;      // 人名
std::vector<string> arKWAction;      // 行为

/// @brief 关键词初始化
void Init_keywords()
{
    Robot.arKWPlacement.push_back("none");
    Robot.arKWPlacement.push_back("none");
    Robot.arKWPlacement.push_back("none");
    Robot.arKWPlacement.push_back("none");
    Robot.arKWPlacement.push_back("none");
    Robot.arKWPlacement.push_back("none");
    // 物品关键词
    Robot.arKWObject.push_back("Water");
    Robot.arKWObject.push_back("Chip");
    Robot.arKWObject.push_back("Sprit");
    Robot.arKWObject.push_back("Cola");
    Robot.arKWObject.push_back("Biscuit");
    Robot.arKWObject.push_back("Bread");
    Robot.arKWObject.push_back("Lays");
    Robot.arKWObject.push_back("Cookie");
    Robot.arKWObject.push_back("Hand wash");
    Robot.arKWObject.push_back("Orange juice");
    Robot.arKWObject.push_back("Dish soap");

    // 人名关键词
    Robot.arKWPerson.push_back("Jack");
    Robot.arKWPerson.push_back("Linda");
    Robot.arKWPerson.push_back("Lucy");
    Robot.arKWPerson.push_back("Grace");
    Robot.arKWPerson.push_back("John");
    Robot.arKWPerson.push_back("Allen");
    Robot.arKWPerson.push_back("Richard");
    Robot.arKWPerson.push_back("Mike");
    Robot.arKWPerson.push_back("Lily");

    // 行为关键词
    Robot.arKWAction.push_back("stand");
    Robot.arKWAction.push_back("down");
    Robot.arKWAction.push_back("lay");
    Robot.arKWAction.push_back("walk");
    Robot.arKWAction.push_back("make telephone");
    Robot.arKWAction.push_back("raise hand");
    Robot.arKWAction.push_back("shake hand");
    Robot.arKWAction.push_back("shake both hands");
    Robot.arKWAction.push_back("smoking");

    cout << "[Init]关键词初始化完成！" << endl;
}

/// @brief 开门检测
/// @param msg Entrance Detect节点传来的消息
void EntranceCB(const std_msgs::String::ConstPtr &msg)
{
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

/// @brief 时钟运行
/// @param e 
void MainCallback(const ros::TimerEvent &e)
{
    if (TimerAct == TimerAct_READY)
    {
        cout << "[Main]正在前往地点：" << arKWPlacement[Robot.nPlaceCount] << endl;
        Robot.Goto(arKWPlacement[Robot.nPlaceCount++]);
        // nPlaceCount++;
        TimerAct = TimerAct_FIND_PERSON;
        sleep(1);
    }

    if (TimerAct == TimerAct_FIND_PERSON)
    {
        if (!Robot.bPeopleFound)
        {
            Robot.SetSpeed(0, 0, 0.1);
        }
        else
        {
            Robot._bFixView = true;
            if (Robot._bFixView_ok == true)
            {
                Robot.ActionDetect();
                Robot._bFixView_ok = false;
            }
            Robot.nPeopleCount++;
            TimerAct = TimerAct_FIND_OBJ;
        }
    }

    if (TimerAct == TimerAct_FIND_OBJ)
    {
        if (!Robot.bObjectFound)
        {
            Robot.SetSpeed(0, 0, 0.1);
        }
        else
        {
            Robot.GrabSwitch(true);
            Robot.nLitterCount++;
            TimerAct = TimerAct_READY;
        }
    }

    if ((Robot.nPeopleCount == 3 && Robot.nLitterCount == 3) && Robot.bPassDone == true)
        nState = STATE_GOTO_EXIT;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    Init_keywords();
    Robot.Init();
    ros::Timer Task_Timer = nh.createTimer(ros::Duration(0.05), &MainCallback);
    cout << "[Main]主节点启动!" << endl;
    nState = STATE_WAIT_ENTR;
    ros::Rate r(10);
    while (ros::ok())
    {
        if (nState == STATE_WAIT_ENTR)
        {
            if (nOpenCount > 20)
            {
                Robot.Enter();
            }
        }

        if (nState == STATE_ACTION)
        {
            Robot.Main();
        }

        if (nState == STATE_GOTO_EXIT)
        {
            cout << "[Main]任务完成 前往退出地点并清空状态" << endl;
            Robot.Exit();
            sleep(5);
            Robot.Reset();
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0; 
}
