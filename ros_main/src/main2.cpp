#include <ros/ros.h>
#include <RobotAct.h> 
/*---------------状态机区---------------*/
#define STATE_READY               0
#define STATE_WAIT_ENTR           1
#define STATE_WAIT_CMD            2                 // 等待指令
#define STATE_ACTION              3
#define STATE_CONFIRM             4
#define STATE_GOTO_EXIT           5                 // 离开房间

#define TimerAct_READY            6                 // 准备
#define TimerAct_FIND_PERSON      7                 // 寻找人
#define TimerAct_FIND_OBJ         8                 // 物品识别
#define TimerAct_GOTO_DUSTBIN     9                 // 去垃圾桶
#define TimerAct_PASS             10
#define TimerAct_GOTO_PEOPLE      11                // 去人航点 
#define TimerAct_CONTACT          12                // 交流
/*---------------初始化区---------------*/


static RobotAct Robot;
/*--------------ROS定义区---------------*/
ros::Timer Task_Timer;
/*---------------全局变量区---------------*/
static int TimerAct = TimerAct_READY;// 任务状态
static int nState = STATE_READY;     // 初始状态
static int nOpenCount = 0;           // 开门延迟
/*---------------数组/容器区---------------*/
std::vector<BBox2D> YOLO_BBOX;                    // 识别结果
std::vector<BBox2D>::const_iterator YOLO_BBOX_IT; // 迭代器
std::vector<BBox2D> recv_BBOX;

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
    if (nState == STATE_WAIT_CMD)
    {
        bool bAction = false;
        if ((Robot.nPeopleCount == 3 && Robot.nLitterCount == 3) && Robot.bPassDone == true)
            nState = STATE_GOTO_EXIT;

        if (TimerAct == TimerAct_READY)
        {
            cout << "[TaskPub]发布任务: 前往地点：" << Rgoobot.arKWPlacement[Robot.nPlaceCount] << endl;
            stAct newAct;
            newAct.nAct = ACT_GOTO;
            newAct.strTarget = Robot.arKWPlacement[Robot.nPlaceCount++];
            Robot.arAct.push_back(newAct);
            if (Robot.nPeopleCount != 3)
            {
                TimerAct = TimerAct_READY;
            }
            TimerAct = TimerAct_FIND_PERSON;
        }

        if (TimerAct == TimerAct_FIND_PERSON && Robot.bArrive ==true)
        {
            if (!Robot.bPeopleFound)
            {
                stAct newAct;
                newAct.nAct = ACT_FIND_PERSON; 
                newAct.strTarget = "FIND_PERSON";
                Robot.arAct.push_back(newAct);
                TimerAct = TimerAct_CONTACT;
            }
            else 
            {
                Timer = TimerAct_CONTACT;
            }
        }


        string object = Robot.FindWord(Robot.strDetect,Robot.arKWObject);
        if (TimerAct == TimerAct_FIND_OBJ)
        {
            if (!Robot.bObjectFound && Robot.bGrabDone != true)
            {
                //Robot.SetSpeed(0, 0, 0.1);
                stAct newAct;
                newAct.nAct = ACT_FIND_OBJ;
                newAct.strTarget = "FIND_OBJ";
                Robot.arAct.push_back(newAct);
            }
            else
            {
                //Robot.GrabSwitch(true);
                stAct newAct;
                newAct.nAct = ACT_GRAB;
                newAct.strTarget = object;
                Robot.arAct.push_back(newAct);
                TimerAct = TimerAct_GOTO_DUSTBIN;
            }
        }

        if (TimerAct == TimerAct_GOTO_DUSTBIN)
        {
            if(Robot.bGrabDone == true)
            {
                stAct newAct;
                newAct.nAct = ACT_GOTO;
                newAct.strTarget = Robot.coord_dustbin;
                Robot.arAct.push_back(newAct);
                TimerAct = TimerAct_PASS;
            }
            else
            {
                TimerAct = TimerAct_FIND_OBJ;
            }
        }

        if(TimerAct == TimerAct_PASS)
        {
            if(Robot.bGrabDone == true && Robot.bPassDone != true)
            {
                stAct newAct;
                newAct.nAct = ACT_PASS;
                newAct.strTarget = true;
                Robot.arAct.push_back(newAct);
                Robot.nLitterCount++;
                bAction = true;
                TimerAct = TimerAct_READY;
            }
        }

        if(bAction == true)
        {
            cout << "[TaskPub]任务确认 展示任务队列..." << endl;
            nState = STATE_CONFIRM;
        }

    }

    if (nState == STATE_CONFIRM)
    {
        Robot.ShowActs();
        nState = STATE_ACTION;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::Subscriber ent_sub = nh.subscribe("/wpb_home/entrance_detect",10,&EntranceCB);
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
                Robot.Speak("我已经进入场地了");
                sleep(3);
                nState = STATE_WAIT_CMD;
            }
        }

        if (nState == STATE_ACTION)
        {
            bool main_finish = Robot.Main();
            //nState = STATE_WAIT_CMD;
            //Robot.Reset();
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

//伪函数
// void VoiceCB(msg)
// {
//     string strListen;
//     strListen = findWord(msg,robot.arkHELLO)
//     if(strListen.lenth() >0 )
//     {
//         arAct newAct;
//         newAct.target = ACT_RECORD;
//         robot.aract.push_back(newAct);
//         bAction = true;
//     }
//     strListen = findword(msg,robot.arkOBj)
//     if(strListen.lengh()>0)
//     {
//         arAct newACt;
//         nweACt.target = obj;
//         robot.aract.push_back(newAct);
//         bCheck = true
//     }

//     if(bcheck == true)
//     {
//         robot.speak(strListen)
//         robot.spaek(确定)
//         if(确定 == false)
//         robot.reset();
//         else
//         nState = STATE_ACTION

//     }
    
// }