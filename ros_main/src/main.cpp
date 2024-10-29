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
#define TimerAct_GRAB             9
#define TimerAct_GOTO_DUSTBIN     10
#define TimerAct_PASS             11
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
    // 物品关键词 == 12
    Robot.arKWObject.push_back("water");
    Robot.arKWObject.push_back("chip");
    Robot.arKWObject.push_back("sprite");
    Robot.arKWObject.push_back("cola");
    Robot.arKWObject.push_back("biscuit");
    Robot.arKWObject.push_back("bread");
    Robot.arKWObject.push_back("lays");
    Robot.arKWObject.push_back("cookie");
    Robot.arKWObject.push_back("handwash");
    Robot.arKWObject.push_back("orange juice");
    Robot.arKWObject.push_back("dishsoap");
    Robot.arKWObject.push_back("shampoo");

    // 人名关键词
    Robot.arKWPerson.push_back("Jack");
    // Robot.arKWPerson.push_back("John");
    // Robot.arKWPerson.push_back("Allen");
    // Robot.arKWPerson.push_back("Richard");
    // Robot.arKWPerson.push_back("Mike");
    // Robot.arKWPerson.push_back("Grace");
    Robot.arKWPerson.push_back("Linda");
    Robot.arKWPerson.push_back("Lily");
    // Robot.arKWPerson.push_back("Lucy");
    // Robot.arKWPerson.push_back("Jennier");

    // 行为关键词
    Robot.arKWAction.push_back("站立");
    Robot.arKWAction.push_back("打电话");
    Robot.arKWAction.push_back("行走");
    Robot.arKWAction.push_back("摔倒");
    Robot.arKWAction.push_back("蹲起");
    Robot.arKWAction.push_back("挥手");
    Robot.arKWAction.push_back("举手");
    Robot.arKWAction.push_back("挥双手");
    Robot.arKWAction.push_back("平躺");
    Robot.arKWAction.push_back("双手交叉");

    Robot.strPerson.push_back("Person");
    Robot.strPerson.push_back("person");
    Robot.strPerson.push_back("People");
    Robot.strPerson.push_back("people");

    Robot.objPlacement.push_back("obj1");
    Robot.objPlacement.push_back("obj2");
    Robot.objPlacement.push_back("obj3");
    Robot.objPlacement.push_back("obj4");
    Robot.objPlacement.push_back("obj5");
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
        if ((RobotAct::nPeopleCount == 3 && RobotAct::nLitterCount == 3) == true)
            nState = STATE_GOTO_EXIT;

        if (TimerAct == TimerAct_READY)
        {
            cout << "[TaskPub]发布任务: 前往地点：" << Robot.arKWPlacement[RobotAct::nPlaceCount] << endl;
            stAct newAct;
            newAct.nAct = ACT_GOTO;
            newAct.strTarget = Robot.arKWPlacement[RobotAct::nPlaceCount++];
            Robot.arAct.push_back(newAct);
            bAction = true;
            TimerAct = TimerAct_FIND_PERSON;
        } 

        if (TimerAct == TimerAct_FIND_PERSON && Robot.bArrive == true)
        {
            if(Robot.GetResult_bPeopleFoundFailed() == true)
            {
                TimerAct = TimerAct_FIND_OBJ; //找人行为失败 -> 找物品
            }
            //Robot.bArrive =false;
            if (!Robot.GetFlag_PeopleFound() && Robot.GetResult_bPeopleFoundFailed() == false) //没找到人 -> 找人//方案一
            {
                cout << "[TaskPub]发布任务: 寻找人物" << endl;
                stAct newAct;
                newAct.nAct = ACT_FIND_PERSON; 
                newAct.strTarget = "FIND_PERSON";
                Robot.arAct.push_back(newAct);
                bAction = true;
            }
            if(Robot.GetFlag_PeopleFound() && Robot.GetResult_bPeopleFoundFailed() == false)
            {
                cout << "[TaskPub]发布任务: 视角修正" << endl;
                // if(!Robot._bFixView_ok)
                // {
                //     Robot._bFixView = true;
                // }
                // else
                // {
                //     Robot._bFixView = false;
                // }
                Robot._bFixView_ok = true; //测试用
                if (Robot.GetResult_FixView() == true) //回调函数中视角修正
                {
                    //Robot._bFixView = false;
                    cout << "[TaskPub]发布任务: 人脸+动作识别" << endl;
                    stAct newAct;
                    newAct.nAct = ACT_ACTION_DETECT;
                    newAct.strTarget = "ACTION_DETECT";
                    Robot.arAct.push_back(newAct);
                    bAction = true;
                    TimerAct = TimerAct_FIND_OBJ;
                    //TimerAct = TimerAct_READY;
                    //Robot.State_Reset();
                    Robot._bFixView_ok = false;
                }
            }
        }
        //cout << "timeact" << TimerAct << endl;
        cout << "action:   "  << Robot.GetResult_ActionDetect()<< endl;
        cout << "face:     "  << Robot.GetResult_FaceRecog()   << endl;
        //cout << "Action"  << RobotAct::bActionDetect << endl;
        string object = Robot.FindWord(Robot.strDetect,Robot.arKWObject);
        if (TimerAct == TimerAct_FIND_OBJ && Robot.GetResult_ActionDetect() == true && Robot.GetResult_FaceRecog() == true
        || TimerAct == TimerAct_FIND_OBJ && Robot.GetResult_bPeopleFoundFailed() == true)
        {
            if(Robot.GetResult_bObjectFoundFailed() == true)
            {
                Robot.Speak("寻找物品失败 前往下一个地点");
                TimerAct = TimerAct_READY; // 没找到物品 -> 进入下一个航点的任务
                Robot.State_Reset();
            }

            if (!Robot.GetFlag_ObjectFound() && !Robot.GetResult_Grab() && Robot.GetResult_bObjectFoundFailed()==false)
            {
                cout << "[TaskPub]发布任务: 物品寻找" << endl;
                stAct newAct;
                newAct.nAct = ACT_FIND_OBJ;
                newAct.strTarget = "FIND_OBJ";
                Robot.arAct.push_back(newAct);
                bAction = true;
            }
            else if(Robot.GetFlag_ObjectFound()==true && Robot.GetResult_bObjectFoundFailed()==false && !Robot.GetResult_Grab())
            {
                cout << "[TaskPub]发布任务: 识别" << endl;
                stAct newAct;
                newAct.nAct = ACT_OBJ_DETECT;
                newAct.strTarget = "OBJ_DETECT"; //预留接口
                Robot.arAct.push_back(newAct);
                bAction = true;
                TimerAct = TimerAct_READY;
                Robot.State_Reset();
            }
        }
        if (TimerAct == TimerAct_GRAB && Robot.GetResult_bObjectFoundFailed() == false)
        {
            if(Robot.GetResult_Grab() == false && Robot.GetFlag_ObjectFound() == true)
            {
                cout <<"[TaskPub]发布任务： 抓取" << endl;
                stAct newAct;
                newAct.nAct = ACT_GRAB;
                newAct.strTarget = object;
                Robot.arAct.push_back(newAct);
                bAction = true;
                TimerAct = TimerAct_GOTO_DUSTBIN;
            }
        }
        if (TimerAct == TimerAct_GOTO_DUSTBIN && Robot.GetResult_Grab() == true)
        {
            if(Robot.GetResult_Grab() == true)
            {
                cout << "[TaskPub]发布任务: 前往垃圾桶" << endl;
                stAct newAct;
                newAct.nAct = ACT_GOTO;
                newAct.strTarget = Robot.coord_dustbin;
                Robot.arAct.push_back(newAct);
                TimerAct = TimerAct_PASS;
                bAction =true;
            }
            else
            {
                //TimerAct = TimcerAct_FIND_OBJ;
                cout <<"等待抓取结束 " << endl;
            }
        }

        if(TimerAct == TimerAct_PASS && Robot.bArrive == true && Robot.GetResult_Grab() == true)
        {
            if(Robot.GetResult_Grab() == true && Robot.GetResult_Pass() != true)
            {
                cout << "[TaskPub]发布任务: 丢弃垃圾" << endl;
                stAct newAct;
                newAct.nAct = ACT_PASS;
                newAct.strTarget = true;
                Robot.arAct.push_back(newAct);
                RobotAct::nLitterCount++;
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
    //ros::Time::init();
    Robot.Init();
    ros::Timer Task_Timer = nh.createTimer(ros::Duration(0.5), &MainCallback);
    cout << "[Main]主节点启动!" << endl;
    nState = STATE_WAIT_ENTR;
    ros::Rate r(10);
    Robot.startFlagUpdater();
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
