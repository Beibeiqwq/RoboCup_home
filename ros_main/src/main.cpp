#include <ros/ros.h>
#include <RobotAct.h> 
/*---------------状态机区---------------*/
#define STATE_READY               0
#define STATE_WAIT_ENTR           1
#define STATE_WAIT_CMD            2
#define STATE_ACTION              3
#define STATE_CONFIRM             4
#define STATE_GOTO_EXIT           5

#define TimerAct_READY            6
#define TimerAct_FIND_PERSON      7
#define TimerAct_FIND_OBJ         8
#define TimerAct_GOTO_DUSTBIN     9
#define TimerAct_PASS             10
/*---------------初始化区---------------*/


static RobotAct Robot;
/*--------------ROS定义区---------------*/
ros::Timer Task_Timer;
/*---------------全局变量区---------------*/
static int TimerAct = TimerAct_READY;// 任务状态
static int nState = STATE_READY;     // 初始状态
static int nOpenCount = 0;           // 开门延迟
static bool bMainFinish = true;
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

    Robot.strPerson.push_back("Person");
    Robot.strPerson.push_back("person");
    Robot.strPerson.push_back("People");
    Robot.strPerson.push_back("people");
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
            cout << "[TaskPub]发布任务: 前往地点：" << Robot.arKWPlacement[Robot.nPlaceCount] << endl;
            stAct newAct;
            newAct.nAct = ACT_GOTO;
            newAct.strTarget = Robot.arKWPlacement[Robot.nPlaceCount++];
            Robot.arAct.push_back(newAct);
            bAction = true;
            TimerAct = TimerAct_FIND_PERSON;
        }

        if (TimerAct == TimerAct_FIND_PERSON && Robot.bArrive == true)
        {
            if (!Robot.bPeopleFound) //在回调函数中实时更新
            {
                cout << "[TaskPub]发布任务: 寻找人物" << endl;
                stAct newAct;
                newAct.nAct = ACT_FIND_PERSON; 
                newAct.strTarget = "FIND_PERSON";
                Robot.arAct.push_back(newAct);
                bAction = true;
            }
            else
            {
                cout << "[TaskPub]发布任务: 视角修正" << endl;
                if(!Robot._bFixView_ok)
                {
                    Robot._bFixView = true;
                }
                else
                {
                    Robot._bFixView = false;
                }
                //Robot._bFixView_ok = true; //测试用
                if (Robot._bFixView_ok == true) //回调函数中视角修正
                {
                    //Robot._bFixView = false;
                    cout << "[TaskPub]发布任务: 动作识别" << endl;

                    stAct newAct;
                    newAct.nAct = ACT_ACTION_DETECT;
                    newAct.strTarget = "ACTION_DETECT";
                    Robot.arAct.push_back(newAct);
                    Robot._bFixView_ok = false;
                    //bAction = true;
                }
                bAction = true;
                TimerAct = TimerAct_FIND_OBJ;
            }
        }
        string object = Robot.FindWord(Robot.strDetect,Robot.arKWObject);
        if (TimerAct == TimerAct_FIND_OBJ && Robot.bActionDetect == true)
        {
            if (!Robot.bObjectFound && !Robot.bGrabDone)
            {
                cout << "[TaskPub]发布任务: 物品寻找" << endl;
                stAct newAct;
                newAct.nAct = ACT_FIND_OBJ;
                newAct.strTarget = "FIND_OBJ";
                Robot.arAct.push_back(newAct);
                //bAction = true;
            }
            else
            {
                cout << "[TaskPub]发布任务: 物品抓取" << endl;
                stAct newAct;
                newAct.nAct = ACT_GRAB;
                newAct.strTarget = object; //预留接口
                Robot.arAct.push_back(newAct);
                TimerAct = TimerAct_GOTO_DUSTBIN;
            }
            bAction = true;
        }

        if (TimerAct == TimerAct_GOTO_DUSTBIN && Robot.bGrabDone == true)
        {
            if(Robot.bGrabDone == true)
            {
                cout << "[TaskPub]发布任务: 前往垃圾桶" << endl;
                stAct newAct;
                newAct.nAct = ACT_GOTO;
                newAct.strTarget = Robot.coord_dustbin;
                Robot.arAct.push_back(newAct);
                TimerAct = TimerAct_PASS;
            }
            else
            {
                //TimerAct = TimcerAct_FIND_OBJ;
                cout <<"等待抓取结束 " << endl;
            }
            bAction =true;
        }

        if(TimerAct == TimerAct_PASS && Robot.bArrive == true && Robot.bGrabDone == true)
        {
            if(Robot.bGrabDone == true && Robot.bPassDone != true)
            {
                cout << "[TaskPub]发布任务: 丢弃垃圾" << endl;
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
                sleep(1);
                Robot.Enter();
                Robot.Speak("我已进入场地");
                sleep(1);
                nState = STATE_WAIT_CMD;
            }
        }

        if (nState == STATE_ACTION)
        {
            bMainFinish = Robot.Main();
            if(bMainFinish == false)
            {
                nState = STATE_WAIT_CMD;
                Robot.Reset();
            }
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
