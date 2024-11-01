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
    // Robot.arKWPlacement.push_back("none");
    // Robot.arKWPlacement.push_back("none");
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
    Robot.arKWPerson.push_back("John");
    Robot.arKWPerson.push_back("Allen");
    Robot.arKWPerson.push_back("Richard");
    Robot.arKWPerson.push_back("Mike");
    Robot.arKWPerson.push_back("Grace");
    Robot.arKWPerson.push_back("Linda");
    Robot.arKWPerson.push_back("Lily");
    Robot.arKWPerson.push_back("Lucy");
    Robot.arKWPerson.push_back("Jennier");

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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::Subscriber ent_sub = nh.subscribe("/wpb_home/entrance_detect",10,&EntranceCB);
    Init_keywords();
    Robot.Init();

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
                nState = STATE_ACTION;
            }
        }

        if (nState == STATE_ACTION)
        {
            int numPeople = 0;
            int nWrong = 0;
            for (auto it = Robot.arKWPlacement.begin() + 1; it != Robot.arKWPlacement.end(); ++it)
            {    
                Robot.bArrive = Robot.Goto(*it);
                Robot.bFinishVoice = false;
                std_msgs::String start;
                start.data = "start";
                
                ros::spinOnce();
                cout << Robot.bPeopleFound << endl;
                DETECT:
                while(1)
                {
                    if(nWrong == 3 )
                    {
                        nState = STATE_GOTO_FIND_OBJ;
                        nWrong = 0;
                        break;
                    }
                    if (Robot.bArrive == true && Robot.bPeopleFound == true && Robot.bFinishVoice == false)//等待交流
                    {
                        Robot.voice_control_pub.publish(start);
                        Robot.Speak("请问你需要什么物品");
                        sleep(5);
                        ros::spinOnce();
                        sleep(2);
                        goto DETECT;
                    }
                    else if (Robot.bArrive == true && Robot.bPeopleFound == true && Robot.bFinishVoice == true)//交流完成，前往下一航点
                    {
                        Robot.Speak("交流完成，前往下一航点");
                        break;
                        advance(it, -1);
                        Robot.arKWPlacement.erase(it + 1);
                        numPeople++;
                    }
                    else if (Robot.bArrive == true && Robot.bPeopleFound == false)//未找到人
                    {
                        nWrong++;
                        break;
                    }
                }

                    if (numPeople == 3)
                    {
                        nState = STATE_GOTO_FIND_OBJ;
                        goto nSTATE_GOTO_FIND_OBJ;
                        //break;
                    }
            
                else
                {
                    continue;
                }
            }    
            
        }
        nSTATE_GOTO_FIND_OBJ:
        if (nState == STATE_GOTO_FIND_OBJ)
        {
            Robot.arKWPlacementObj.push_back("A");
            Robot.arKWPlacementObj.push_back("B");
            Robot.arKWPlacementObj.push_back("C");
            Robot.arKWPlacementObj.push_back("D");
            Robot.arKWPlacementObj.push_back("E");
            Robot.arKWPlacementObj.push_back("F");
            for(auto it = Robot.arKWPlacementObj.begin(); it != Robot.arKWPlacementObj.end(); ++it)
            {
                Robot.bArrive = Robot.Goto(*it);
                Robot.Speak("查看地点");
                // if (Robot.FindWord_Yolo_MAP(Robot.YOLO_BBOX, Robot.objName) != std::string::npos)
                // {
                //     Robot.Speak("哈哈哈哈和");
                // }
                // else
                // {

                // }
                

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
