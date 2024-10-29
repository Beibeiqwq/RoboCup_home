#include "RobotAct.h"
/**********************************************************/
/*                       定义区                            */
/**********************************************************/

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

static string strToSpeak = "";
static string strKeyWord = "";

bool RobotAct::bActionDetect = false;       // 动作识别开关
int  RobotAct::nPeopleCount = 0;            // 完成动作识别计数
int  RobotAct::nLitterCount = 0;            // 完成垃圾丢弃计数
int  RobotAct::nPlaceCount  = 1;            // 房间导航地点计数
int  RobotAct::nObjPlaceCount = 1;          // 物品导航地点计数

static float grab_y_offset       = 0.0f;    //抓取前，对准物品，机器人的横向位移偏移量
static float grab_lift_offset    = 0.0f;    //手臂抬起高度的补偿偏移量
static float grab_forward_offset = 0.0f;    //手臂抬起后，机器人向前抓取物品移动的位移偏移量
static float grab_gripper_value  = 0.032;   //抓取物品时，手爪闭合后的手指间距

/**********************************************************/
/*                       初始化                            */
/**********************************************************/

/// @brief 构造函数
RobotAct::RobotAct()
{
    nCurActIndex  =  0;
    nCurActCode   = -1;
    _nActionStage =  1;

    strListen = "";
    bGrabDone = false;
    bPassDone = false;
    strFace = "";
    Object_map = 
    {
        {"water","水"},
        {"biscuit","饼干"},
        {"lays","乐事薯片"},
        {"chips","薯片"},
        {"cookie","曲奇"},
        {"handwash","洗手液"},
        {"dishsoap","洗洁精"},
        {"sprite","雪碧"},
        {"cola","可乐"},
        {"orange juice","芬达"},
        {"shampoo","洗发水"},
        {"bread","面包"}
    };
}

/// @brief 析构函数
RobotAct::~RobotAct()
{
}

/// @brief 机器人初始化
void RobotAct::Init()
{
    ros::NodeHandle n("~");
    /*---------------参数导入区---------------*/
    n.param<string>("name", _name_yaml, "default");
    n.param<string>("enter", _coord_cmd, "cmdA");
    n.param<string>("place1", arKWPlacement[1], "living room");
    n.param<string>("place2", arKWPlacement[2], "kitchen");
    n.param<string>("place3", arKWPlacement[3], "bedroom");
    n.param<string>("place4", arKWPlacement[4], "dining room");
    n.param<string>("obj1", objPlacement[1], "obj living room");
    n.param<string>("obj2", objPlacement[2], "obj kitchen");
    n.param<string>("obj3", objPlacement[3], "obj bedroom");
    n.param<string>("obj4", objPlacement[4], "obj dining room");
    n.param<string>("dustbin",coord_dustbin,"dustbinA");
    n.param<string>("exit", _coord_exit, "exitA");
    n.param<float> ("PID_Forward", _PID_Forward, 0.0002);
    n.param<float> ("PID_Turn", _PID_Turn, 0.0003);
    cout << "参数初始化完毕" << endl;
    /*---------------ROS初始化---------------*/
    sub_yolo         = n.subscribe("/yolo_bbox_2d", 5, &RobotAct::YOLOV5Callback, this);
    sub_pose         = n.subscribe("/Openpose", 1, &RobotAct::OpenPoseCallback, this);
    sub_face         = n.subscribe("/FaceDetect", 5, &RobotAct::FaceRecogCallback, this);
    grab_result_sub  = n.subscribe<std_msgs::String>("/wpb_home/grab_result", 30, &RobotAct::GrabResultCallback, this);
    pass_result_sub  = n.subscribe<std_msgs::String>("/wpb_home/pass_result", 30, &RobotAct::PassResultCallback, this);
    client_speak     = n.serviceClient<robot_voice::StringToVoice>("/str2voice");
    cliGetWPName     = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    chatter_server_  = n.advertiseService("/human_chatter", &RobotAct::ChatterCallback, this);
    speak_pub        = n.advertise<sound_play::SoundRequest>("/robotsound", 20);
    speed_pub        = n.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    yolo_pub         = n.advertise<std_msgs::String>("/yolov5/cmd", 20);
    behaviors_pub    = n.advertise<std_msgs::String>("/wpb_home/behaviors", 30);
    add_waypoint_pub = n.advertise<waterplus_map_tools::Waypoint>("/waterplus/add_waypoint", 1);
    mani_ctrl_pub    = n.advertise<sensor_msgs::JointState>("/wpb_home/mani_ctrl", 30);
    /*--------------机械臂初始化--------------*/
    mani_ctrl_msg.name.resize(2);
    mani_ctrl_msg.position.resize(2);
    mani_ctrl_msg.velocity.resize(2);
    mani_ctrl_msg.name[0] = "lift";
    mani_ctrl_msg.name[1] = "gripper";
    mani_ctrl_msg.position[0] = 0;
    mani_ctrl_msg.velocity[0] = 0.5;     //升降速度(单位:米/秒)
    mani_ctrl_msg.position[1] = 0.16;
    mani_ctrl_msg.velocity[1] = 5;       //手爪开合角速度(单位:度/秒)
    /*---------------主程序区域---------------*/
    cout << "[Init]请检查程序参数...." << endl;
    Parameter_Check();
    cout << "[Init]键入任意数字开始.... 按CTRL+Z退出" << endl;
    cin >> _check_flag;
}

/**********************************************************/
/*                       状态机                            */
/**********************************************************/
static int nLastActCode = -1;
static geometry_msgs::Twist vel_cmd;
bool RobotAct::Main()
{
    // 任务个数
    int nNumOfAct = arAct.size();
    // 结束判定
    if (nCurActIndex >= nNumOfAct)
    {
        return false;
    }
    // 语音识别的关键词 
    int nKeyWord = -1;
    // 当前任务状态
    nCurActCode = arAct[nCurActIndex].nAct;
    // nCurActIndex == 当前任务ID
    // nLastActCode == 上一个任务ID
    switch (nCurActCode)
    {
    case ACT_GOTO:
        if (nLastActCode != ACT_GOTO)
        {
            string StrGoto = arAct[nCurActIndex].strTarget;
            printf("[RobotAct] %d - Find %s\n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
            bArrive = Goto(StrGoto);
            nCurActIndex++;
        }
        break;

    case ACT_GRAB:
        if (nLastActCode != ACT_GRAB)
        {
            printf("[RobotAct] %d - Grab %s\n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
            bGrabDone = false;
            GrabSwitch(true);
        }
        if (bGrabDone == true)
        {
            printf("[RobotAct] %d - Grab %s done!\n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
            GrabSwitch(false);
            nCurActIndex++;
        }
        break;

    case ACT_PASS:
        if (nLastActCode != ACT_PASS)
        {
            printf("[RobotAct] %d - Pass %s\n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
            bPassDone = false;
            PassSwitch(true);
        }
        if (bPassDone == true)
        {
            printf("[RobotAct] %d - Pass %s done! \n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
            PassSwitch(false);
            nCurActIndex++;
        }
        break;


    case ACT_ADD_WAYPOINT:
        if (nLastActCode != ACT_ADD_WAYPOINT)
        {
            printf("[RobotAct] %d - Add waypoint %s \n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
            AddNewWaypoint(arAct[nCurActIndex].strTarget);
            nCurActIndex++;
        }
        break;

    case ACT_FIND_PERSON://待优化
        if (nLastActCode != ACT_FIND_PERSON)
        {
            double turn_speed = 0.3;
            double turn_angle = M_PI / 4;
            double rotate_duration = turn_angle / turn_speed;
            ros::Time start_time = ros::Time::now();
            ros::Duration timeout(15.0);
            ros::Duration turn_time(2.0);
            //ros::spinOnce();
            while (ros::ok())
            {
                //等待标志位更新
                ros::spinOnce();
                if (GetFlag_PeopleFound())
                {
                    SetSpeed(0, 0, 0);
                    Speak("找到人啦");
                    bPeopleFound_failed = false;
                    nCurActIndex++;
                    break;
                }
                Speak("未识别到人 进入找人行为");
                //第一次正转
                SetSpeed(0, 0, turn_speed);
                //ros::Duration(rotate_duration).sleep();
                //SetSpeed(0, 0, 0);
                ros::spinOnce();
                //判断是否找到人
                if (GetFlag_PeopleFound())
                {
                    SetSpeed(0, 0, 0);
                    Speak("找到人啦");
                    bPeopleFound_failed = false;
                    nCurActIndex++;
                    break;
                }
                //第二次反转(可以不需要反转？)
                if ((ros::Time::now() - start_time).toSec() >= turn_time.toSec())
                {
                    SetSpeed(0, 0, 0);
                }
                SetSpeed(0, 0, -turn_speed);

                //ros::Duration(rotate_duration*2).sleep();
                //SetSpeed(0, 0, 0);
                ros::spinOnce();
                //判断是否找到人
                if(GetFlag_PeopleFound())
                {
                    SetSpeed(0, 0, 0);
                    Speak("找到人啦");
                    bPeopleFound_failed = false;
                    nCurActIndex++;
                    break;
                }

                if ((ros::Time::now() - start_time).toSec() >= timeout.toSec())
                {
                    Speak("找人失败");
                    cout << "找人失败" << endl;
                    bPeopleFound_failed = true;
                    nCurActIndex++;
                    break; // 超时，退出循环
                }

                // else if(!GetFlag_PeopleFound())
                // {
                //     //方案一 转过一定的角度后再找人
                //     SetSpeed(0, 0, turn_speed);
                //     if ((ros::Time::now() - start_time).toSec() >= rotate_duration)
                //     {
                //         SetSpeed(0, 0, 0);//正转
                //     }
                //     //sleep(1);
                //     SetSpeed(0, 0, -turn_speed);
                //     //start_time = ros::Time::now();
                //     if((ros::Time::now() - start_time).toSec() >= 2*rotate_duration)
                //     {
                //         SetSpeed(0, 0, 0);//反转
                //     }

                //     if((ros::Time::now() - start_time).toSec() >= timeout.toSec())
                //     {
                //         Speak("找人失败");
                //         cout << "找人失败" << endl;
                //         bPeopleFound_failed = true;
                //         break;
                //     }
                //     else
                //     {
                //         bPeopleFound_failed = false;
                //         break;
                //     }
                    //break;
                    // turn_angle -= M_PI / 6;
                    // if (turn_angle <= 0)
                    // {
                    //     break;
                    // }
                // }
            }

        }
        break;

    case ACT_FIND_OBJ:
        if (nLastActCode != ACT_FIND_OBJ)
        {
            double turn_speed = 0.2;
            double turn_angle = M_PI / 6;
            double rotate_duration = turn_angle / turn_speed;
            //cout <<"[test]rotate_duration" << rotate_duration << endl;
            Speak("未识别到物品 进入找物品行为 前往第一个物品航点"); //测试
            Goto(objPlacement[nObjPlaceCount++]);
            // ros::Time::init();
            ros::Time start_time = ros::Time::now();
            ros::Duration timeout(15.0);
            while (ros::ok())
            {
                ros::spinOnce();
                if (GetFlag_ObjectFound())
                {
                    SetSpeed(0, 0, 0);
                    bObjectFound_failed = false;
                    nCurActIndex++;
                    break;
                }

                // 进行转动
                SetSpeed(0, 0, turn_speed);
                //ros::Duration(rotate_duration).sleep();
                //SetSpeed(0, 0, 0);
                ros::spinOnce();

                if (GetFlag_ObjectFound())
                {
                    SetSpeed(0, 0, 0);
                    bObjectFound_failed = false;
                    nCurActIndex++;
                    break;
                }

                SetSpeed(0, 0, -turn_speed);
                //ros::Duration(1.5*rotate_duration).sleep();
                //SetSpeed(0, 0, 0);
                ros::spinOnce();

                if (GetFlag_ObjectFound())
                {
                    SetSpeed(0, 0, 0);
                    bObjectFound_failed = false;
                    nCurActIndex++;
                    break;
                }

                if ((ros::Time::now() - start_time).toSec() >= timeout.toSec())
                {
                    cout << "找物品失败" << endl;
                    Speak("找物品失败");
                    bObjectFound_failed = true;
                    nCurActIndex++;
                    break;
                }
            }
            // if(GetFlag_ObjectFound())
            // {
            //     nCurActIndex++;
            // }
            // nCurActIndex++;
        }
        break;

    case ACT_ACTION_DETECT:
        if (nLastActCode != ACT_ACTION_DETECT)
        {
            if (GlobalbPeopleFound == true)
            {
                //可添加视角修正

                FaceDetect();
                bOpenpose = true; //开启Openpose回调开关
                // if(bFaceDetect == true)
                // {
                //     bOpenpose = true;
                //     ActionDetect();
                //     nCurActIndex++;
                // }
                sleep(1);
                ActionDetect1();
                bOpenpose = false;
                nCurActIndex++;
            }
        }
        break;

    default:
        break;
    }
    // 标记当前行为
    nLastActCode = nCurActCode;
    return true;
}
// 状态重置
void RobotAct::Reset()
{
    strToSpeak = "";
    nCurActIndex = 0;
    nLastActCode = 0;
    //bArrive = false;
    arAct.clear();
}

/**********************************************************/
/*                       回调函数                          */
/**********************************************************/

/// @brief 抓取结果
/// @param res 
void RobotAct::GrabResultCallback(const std_msgs::String::ConstPtr &res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if (nFindIndex >= 0)
    {
        bGrabDone = true;
    }
}

/// @brief 递给结果
/// @param res 
void RobotAct::PassResultCallback(const std_msgs::String::ConstPtr &res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if (nFindIndex >= 0)
    {
        bPassDone = true;
    }
}

/// @brief YOLO回调
/// @param msg
void RobotAct::YOLOV5Callback(const wpb_yolo5::BBox2D &msg)
{
    //cout << "[YOLOV5CB]:接收到Yolov5数据" << endl;
    YOLO_BBOX.clear();
    recv_BBOX.clear();
    bPeopleFound = false;
    bObjectFound = false;
    string Peoplename = "";
    string Objectname = "";
    int nNum = msg.name.size();
    bool bAction = false;
    if (nNum > 0)
    {
        // std::vector<BBox2D> recv_BBOX; // 收到的物品
        BBox2D box_object; // bbox格式object 存入收到的msg
        for (int i = 0; i < nNum; i++)
        {
            box_object.name = msg.name[i];               // 识别到的名字
            box_object.left = msg.left[i];               // x_min
            //cout << "left:" << msg.left[i] << endl;
            box_object.right = msg.right[i];             // x_max
            //cout << "right" << msg.right[i] << endl;
            box_object.top = msg.top[i];                 // y_min
            //cout << "top" << msg.top[i] << endl;
            box_object.bottom = msg.bottom[i];           // y_max
            //cout << "bottom" << msg.bottom[i] << endl;
            box_object.probability = msg.probability[i]; // 置信度
            recv_BBOX.push_back(box_object);
            //strDetect = msg.name[i];
            Peoplename = FindWord(recv_BBOX[i].name, strPerson);
            //cout << "Peoplename =====" << Peoplename << endl;
            Objectname = FindWord(recv_BBOX[i].name, arKWObject);
            //Kinect2 QHD发布的图像 像素为960*540 Kinect2 HD发布的图像 像素为1920*1080
            if (Peoplename.length() > 0)
            {
                //cout << "进入if Peoplename.length()>0" << endl;
                bPeopleFound = true;
                nYoloPeople = i;
                //updateFlagbPeopleFound();
                //cout << "bPeopleFound = true" << endl;
                _nImgHeight = box_object.top - box_object.bottom;
                _nImgWidth = box_object.right - box_object.left;
                _nTargetX = 1024;
                _nTargetY = 540;
            }
            // if(Peoplename.length() == 0)
            else
            {
                bPeopleFound = false;
                //updateFlagbPeopleFound();
                //cout << "bPeopleFound = false" << endl;
            }
            if(Objectname.length() > 0)
            {
                strDetect = msg.name[i];//考虑替换成容器 塞多个物品？
                bObjectFound = true;
                //cout << "bObjectFound = true" << endl;
                //bPeopleFound = false;
                //updateFlagbObjectFound();
            }
            // if(Objectname.length() == 0)
            else
            {
                bObjectFound = false;
                //cout << "bObjectFound = false" << endl;
                //updateFlagbObjectFound();
            }
        }
        YOLO_BBOX = recv_BBOX; // 存入object
    }

    ///@brief 位姿修正
    if (_bFixView == true)
    {
        cout << "[FixView]位姿修正开始...." << endl;
        _fVelForward = _fVelTurn = 0;
        if (nNum != 0)
        {
            cout << "进入nNum ==0 "<< endl;
            if (YOLO_BBOX[nYoloPeople].left != 0 && YOLO_BBOX[nYoloPeople].top != 0)
            {
                cout << "标定位置信息为:" << "x:" << YOLO_BBOX[nYoloPeople].left << "y:" << YOLO_BBOX[nYoloPeople].top << endl;
                if(YOLO_BBOX[nYoloPeople].left >= 700 && YOLO_BBOX[nYoloPeople].right <= 1400)
                {
                    _fVelForward = _fVelForward = 0;
                    SetSpeed(VelFixed(_fVelForward, _vel_max), 0, VelFixed(_fVelTurn, _vel_max));
                    _bFixView_ok = true;
                    _bFixView = false;
                }
                
                
                if (YOLO_BBOX[nYoloPeople].left < 700)
                {
                    // fVelForward = (nImgHeight / 2 - nTargetY) * PID_Forward;
                    _fVelTurn = (_nImgWidth / 2 - _nTargetY) * _PID_Turn;
                }
                else if (YOLO_BBOX[nYoloPeople].left > 1400)
                {
                    _fVelTurn = (_nImgWidth / 2 - _nTargetY) * _PID_Turn;
                }
                // else if (YOLO_BBOX[nYoloPeople].top < 540)
                // {
                //     _fVelForward = (_nImgHeight / 2 - _nTargetY) * _PID_Forward;
                // }
                // else if (YOLO_BBOX[nYoloPeople].top > 540)
                // {
                //     _fVelForward = (_nImgHeight / 2 - _nTargetY) * _PID_Forward;
                // }
                else
                {
                    _fVelForward = _fVelForward = 0;
                }
            }
        }
        SetSpeed(VelFixed(_fVelForward, _vel_max), 0, VelFixed(_fVelTurn, _vel_max));
        _bFixView_ok = true;
        _bFixView = false;
    }
}

/// @brief OpenPose回调
void RobotAct::OpenPoseCallback(const std_msgs::String::ConstPtr &msg)
{
    string strAction;
    string strOpenpose = msg->data;
    if(bOpenpose == true)
    {
        action_counts[msg->data]++;
    }
    else if (bOpenpose == false)
    {
        action_counts.clear();
    }
    cout << "[OpenPoseCB]接收到OpenPose数据:" << strOpenpose << endl;
    // if (bOpenpose == true) 
    // {
    //     strAction = FindWord(strOpenpose, arKWAction);
    //     if (strAction.length() > 0)
    //     {
    //         GlobalstrAction = strAction;
    //     }
    //     else
    //     {
    //         GlobalstrAction = "";
    //     }
    // }
}

/// @brief 人脸识别
/// @param msg 
void RobotAct::FaceRecogCallback(const std_msgs::String::ConstPtr& msg)
{
    strFace = msg->data;
    //cout << "[FaceRecogCB]接收到人脸识别数据" << strFace << endl;
}

/// @brief 机器人对话
/// @param req 
/// @param resp 
/// @return 
bool RobotAct::ChatterCallback(robot_voice::StringToVoice::Request &req, robot_voice::StringToVoice::Response &resp)
{
    if(bKeyVoice == false)
        return false;
    printf("识别到: %s\n", req.data.c_str());
    std::string voice_txt = req.data;
    if (voice_txt.find("抓取") != std::string::npos)
    {
        Speak("请在我抬起手臂后将物品放置在我的抓取区域");
        Raise_arm();
        sleep(5);
        Speak("手臂已抬起，请确认是否放置完成");
    }
    if (voice_txt.find("确认") != std::string::npos)
    {
        Speak("好的，我将进行抓取 请小心");
        sleep(2);
        Grab_arm();
    }
    resp.success = true;
    return resp.success;
}

/**********************************************************/
/*                   程序功能区                             */
/**********************************************************/

/// @brief 程序参数打印
void RobotAct::Parameter_Check()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>> Parameter_Check <<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "Yaml Name:"   << _name_yaml  << endl;
    cout << "Enter Coord:" << _coord_cmd  << endl;
    cout << "Exit  Coord:" << _coord_exit << endl;
    cout << "Place1:" << arKWPlacement[1] << endl;
    cout << "Place2:" << arKWPlacement[2] << endl;
    cout << "Place3:" << arKWPlacement[3] << endl;
    cout << "Place4:" << arKWPlacement[4] << endl;
    cout << "obj1:"   << objPlacement[1]  << endl;
    cout << "obj2:"   << objPlacement[2]  << endl;
    cout << "obj3:"   << objPlacement[3]  << endl;
    cout << "obj4:"   << objPlacement[4]  << endl;
    cout << "PID_ForWard:" << _PID_Forward<< endl;
    cout << "PID_Turn:"    << _PID_Turn   << endl;
    cout << ">>>>>>>>>>>>>>>>>>>> Please check the parameter <<<<<<<<<<<<<<<<<<" << endl;
}

/// @brief 状态打印
/// @param inAct 
/// @return 
string ActionText(stAct *inAct)
{
    string ActText = "";
    if (inAct->nAct == ACT_GOTO)
    {
        ActText = "去往地点 ";
        ActText += inAct->strTarget;
    }
    if (inAct->nAct == ACT_FIND_OBJ)
    {
        ActText = "搜索物品 ";
        ActText += inAct->strTarget;
    }
    if (inAct->nAct == ACT_GRAB)
    {
        ActText = "抓取物品 ";
        ActText += inAct->strTarget;
    }
    if (inAct->nAct == ACT_PASS)
    {
        ActText = "把物品递给 ";
        ActText += inAct->strTarget;
    }
    if (inAct->nAct == ACT_SPEAK)
    {
        ActText = "说话 ";
        ActText += inAct->strTarget;
    }
    if (inAct->nAct == ACT_OBJ_DETECT)
    {
        ActText = "物品识别 ";
        ActText += inAct->strTarget;
    }
    if (inAct->nAct == ACT_ADD_WAYPOINT)
    {
        ActText = "添加航点 ";
        std::ostringstream stringStream;
        // stringStream << inAct->fFollowDist;
        std::string retStr = stringStream.str();
        ActText += retStr;
    }
    if (inAct->nAct == ACT_FIND_PERSON)
    {
        ActText = "寻找人物 ";
        ActText += inAct->strTarget;
    }
    if (inAct->nAct == ACT_ACTION_DETECT)
    {
        ActText = "动作识别 ";
        ActText += inAct->strTarget;
    }
    return ActText;
}

/// @brief 展示状态
void RobotAct::ShowActs()
{
    printf("\n*********************************************\n");
    printf("显示行为队列:\n");
    int nNumOfAct = arAct.size();
    stAct tmpAct;
    for (int i = 0; i < nNumOfAct; i++)
    {
        tmpAct = arAct[i];
        string act_txt = ActionText(&tmpAct);
        printf("行为 %d : %s\n", i + 1, act_txt.c_str());
    }
    printf("*********************************************\n\n");
}

/// @brief 说话函数（废弃）
/// @return 
string RobotAct::GetToSpeak()
{
    string strRet = strToSpeak;
    strToSpeak = "";
    return strRet;
}

/// @brief 寻找关键词 YOLO版
/// @param inSentence 传入的句子 -- YoloV5 Node中
/// @param arWord     关键词    -- Init_Keywords中
/// @return 得到的关键词
string RobotAct::FindWord_Yolo(vector<BBox2D> &YOLO_BBOX, vector<string> &arWord)
{
    string strRes = "";
    int nNum = arWord.size();
    for (const auto &bbox : YOLO_BBOX)
    {
        for (int j = 0; j < nNum; j++)
        {
            int tmpIndex = bbox.name.find(arWord[j]);
            if (tmpIndex >= 0)
            {
                strRes = arWord[j];
                break;
            }
        }
    }
    return strRes;
}

/// @brief 寻找关键词 字符串版
/// @param inSentence
/// @param arWord
/// @return 字符串
string RobotAct::FindWord(string inSentence, vector<string> &arWord)
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

void RobotAct::State_Reset()
{
    bArrive       = false;
    bActionDetect = false;
    bFaceDetect   = false;
    bObjectFound_failed = false;
    bPeopleFound_failed = false;
    cout << "[State_Reset] 重置状态" << endl;
}

/// @brief 标志位更新（人）
void RobotAct::updateFlagbPeopleFound()
{
    ros::Rate update_rate      = ros::Rate(1);
    // GlobalbPeopleFound = bPeopleFound;
    // cout << "[bPeopleFound]Flag updated to " << GlobalbPeopleFound << endl;
    // update_rate.sleep(); //2s更新一次
    while (ros::ok())
    {
        GlobalbPeopleFound = bPeopleFound;
        std::cout << "[bPeopleFound]Flag updated to " << GlobalbPeopleFound << endl;
        ros::spinOnce();
        update_rate.sleep(); // 1s更新一次
    }
}

/// @brief 标志位更新（物）
void RobotAct::updateFlagbObjectFound()
{
    //update_rate      = ros::Rate(0.5);
    ros::Rate update_rate      = ros::Rate(1);
    while (ros::ok())
    {
        GlobalbObjectFound = bObjectFound;
        cout << "[bObjectFound]Flag updated to " << GlobalbObjectFound << endl;
        ros::spinOnce();
        update_rate.sleep(); // 1s更新一次
    }
}

void RobotAct::startFlagUpdater()
{
    std::thread peopleUpdaterThread(&RobotAct::updateFlagbPeopleFound, this);
    std::thread objectUpdaterThread(&RobotAct::updateFlagbObjectFound, this);

    peopleUpdaterThread.detach(); // 分离人标志位更新线程
    objectUpdaterThread.detach(); // 分离物体标志位更新线程
}
/**********************************************************/
/*                   机器人功能区                           */
/**********************************************************/

/// @brief 航点添加
/// @param inStr 
void RobotAct::AddNewWaypoint(string inStr)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("[lookupTransform] %s", ex.what());
        return;
    }

    float tx = transform.getOrigin().x();
    float ty = transform.getOrigin().y();
    tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(transform.getRotation(), tf::Point(tx, ty, 0.0)), ros::Time::now(), "map");
    geometry_msgs::PoseStamped new_pos;
    tf::poseStampedTFToMsg(p, new_pos);

    waterplus_map_tools::Waypoint new_waypoint;
    new_waypoint.name = inStr;
    new_waypoint.pose = new_pos.pose;
    add_waypoint_pub.publish(new_waypoint);

    ROS_WARN("[New Waypoint] %s ( %.2f , %.2f )", new_waypoint.name.c_str(), tx, ty);
}

/// @brief 机器人速度发布
/// @param inVx 向前/后移动速度
/// @param inVy 向左/右移动速度
/// @param inTz 旋转角速度
void RobotAct::SetSpeed(float inVx, float inVy, float inTz)
{
    std::thread speed_thread(&RobotAct::SpeedThread, this, inVx, inVy, inTz);
    speed_thread.detach(); // 分离线程，允许它独立运行
}

void RobotAct::SpeedThread(float inVx, float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = VelFixed(inVx, _vel_max);
    vel_cmd.linear.y = VelFixed(inVy, _vel_max);
    vel_cmd.angular.z = VelFixed(inTz, _vel_max);
    cout << "设置机器人速度为" << "X:" << inVx << "Y:" << inVy << "Z:" << inTz << endl;
    speed_pub.publish(vel_cmd);
}



/// @brief 机器人速度修正
/// @param inVel 输入速度
/// @param inMax 最大速度
/// @return 修正后的速度
float RobotAct::VelFixed(float inVel, float inMax)
{
    float retVel = inVel;
    if (retVel > inMax)
        retVel = inMax;
    if (retVel < -inMax)
        retVel = -inMax;
    return retVel;
}

/// @brief 航点导航
/// @param inStr 航点名称
/// @return Ture -> 导航成功 False ->导航失败
bool RobotAct::Goto(string inStr)
{
    string strGoto = inStr;
    srvName.request.name = strGoto;
    if (cliGetWPName.call(srvName))
    {
        std::string name = srvName.response.name;
        float x = srvName.response.pose.position.x;
        float y = srvName.response.pose.position.y;
        ROS_INFO("[Goto]Get_wp_name: name = %s (%.2f,%.2f)", strGoto.c_str(), x, y);

        MoveBaseClient ac("move_base", true);
        if (!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("[Goto]The move_base action server is no running. action abort...");
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
                ROS_INFO("[Goto]Arrived at %s!", strGoto.c_str());
                return true;
            }
            else
            {
                ROS_INFO("[Goto]Failed to get to %s ...", strGoto.c_str());
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

/// @brief 进门
void RobotAct::Enter()
{
    cout << "[RobotAct]正在前往进门地点...." << endl;
    Goto(_coord_cmd);
}

/// @brief 出门
void RobotAct::Exit()
{
    cout << "[RobotAct]正在前往出门地点...." << endl;
    Goto(_coord_exit);
}

/// @brief 抓取开关
/// @param inActive 
void RobotAct::GrabSwitch(bool inActive)
{
    //std_msgs::String behavior_msg;
    if (inActive == true)
    {
        //behavior_msg.data = "grab start";
        //behaviors_pub.publish(behavior_msg);
        bKeyVoice = true;
    }
    else
    {
        bKeyVoice = false;
        //behavior_msg.data = "grab stop";
        //behaviors_pub.publish(behavior_msg);
    }
}

/// @brief 递给开关
/// @param inActive 
void RobotAct::PassSwitch(bool inActive)
{
    //std_msgs::String behavior_msg;
    if (inActive == true)
    {
        // behavior_msg.data = "pass start";
        // behaviors_pub.publish(behavior_msg);
        Pass_arm();
        bPassDone = true;
    }
    else
    {
        // behavior_msg.data = "pass stop";
        // behaviors_pub.publish(behavior_msg);
    }
}

/// @brief 机器人说话（考虑替换为xfyun）
/// @param answer_txt 说话内容
void RobotAct::Speak(const std::string &answer_txt)
{
    robot_voice::StringToVoice::Request req;
    robot_voice::StringToVoice::Response resp;
    req.data = answer_txt;
    bool ok = client_speak.call(req, resp);
    if (ok)
    {
        //printf("[Speak]send str2voice service success: %s", req.data.c_str());
        cout << "[RobotAct]发送语音任务到 'str2voice' " << req.data << endl;
    }
    else
    {
        ROS_ERROR("[RobotAct]Speak Error");
    }
}

/// @brief 机械臂抬起
void RobotAct::Raise_arm()
{
    cout << "正在抬起手臂...." << endl;
    // mani_ctrl_msg.position[0] = 0.5;
    // mani_ctrl_msg.position[1] = 0.16;
    mani_ctrl_msg.position[0] = 1.0;
    mani_ctrl_msg.position[1] = -0.1;
    mani_ctrl_pub.publish(mani_ctrl_msg);
    ROS_WARN("[MANI_CTRL] lift= %.2f  gripper= %.2f ", mani_ctrl_msg.position[0], mani_ctrl_msg.position[1]);
}

/// @brief 机械臂抓取
void RobotAct::Grab_arm()
{
    cout << "正在抓取..." << endl;
    mani_ctrl_msg.position[1] = grab_gripper_value; // 抓取物品手爪闭合宽度
    mani_ctrl_pub.publish(mani_ctrl_msg);
    bGrabDone = true;
}

void RobotAct::Pass_arm()
{
    cout << "正在递出..." << endl;
    mani_ctrl_msg.position[1] = 0.16; // 递出物品手爪张开宽度
    mani_ctrl_pub.publish(mani_ctrl_msg);
    bPassDone = true;
}

/**********************************************************/
/*                   机器人任务区                           */
/**********************************************************/

/// @brief 动作识别
void RobotAct::ActionDetect()
{
    int StableCount     = 0;
    int StableThreshold = 10;
    string strCurrentAction = "";
    string strLastAction    = "";
    string strStableAction  = "";
    std::vector<std::string> actions;

    cout << "[ActionDetect]动作识别开始...." << endl;
    Speak("动作识别开始 请开始你的第一个动作");
    //sleep(2);
    //GlobalstrAction = "站立";
    while (ros::ok())
    {
        ros::spinOnce();
        strCurrentAction = getActionFromOpenpose();
        actions.push_back(strCurrentAction);

        if(actions.size() > 1 && actions.back() == actions[actions.size() - 2])
        {
            StableCount++;
        }
        else
        {
            StableCount = 0;
        }

        if (StableCount > StableThreshold)
        {
            std::cout << "识别到连续动作：" << actions.back() << "，次数：" << StableCount << endl;
            strStableAction = actions.back();
            break;
        }
        ros::spinOnce();
        sleep(0.2);

    }
    
    if (_nActionStage == 1)
    {
        Speak("识别到第一个动作");
        Speak(strStableAction);
        strLastAction = strStableAction;
        _nActionStage = 2;
    }
    if(_nActionStage == 2)
    {
        Speak("你可以展示下一个动作了");
        sleep(2);
        _nActionStage = 3;
    }
    if (_nActionStage == 3)
    {
        actions.clear();
        int StableCount = 0;
        const int StableThreshold = 10;
        // std::string strCurrentAction;
        // std::string strStableAction;
        strCurrentAction = "";
        strStableAction  = "";
        bool needRecognition = true; // 用于控制识别循环

        while (ros::ok())
        {
            if (needRecognition)
            {
                ros::spinOnce();
                strCurrentAction = getActionFromOpenpose();
                actions.push_back(strCurrentAction);

                if (actions.size() > 1 && actions.back() == actions[actions.size() - 2])
                {
                    StableCount++;
                }
                else
                {
                    StableCount = 0;
                }

                if (StableCount > StableThreshold)
                {
                    //std::cout << "识别到连续动作：" << actions.back() << "，次数：" << StableCount << std::endl;
                    strStableAction = actions.back();
                    //_nActionStage = 4;
                    //std::cout << "_nActionStage=" << _nActionStage << std::endl;
                    needRecognition = false; // 停止识别
                }
            }
            else
            {
                //sleep(1);
                // 如果需要再次识别，则重新设置 needRecognition = true;
                if (strStableAction == strLastAction)
                {
                    actions.clear();
                    int StableCount = 0;
                    strCurrentAction = "";
                    strStableAction  = "";
                    needRecognition = true; // 如果不稳定，重新开始识别
                }
                else
                {
                    _nActionStage = 4;
                    break;
                }
            }
            ros::spinOnce();
            sleep(0.2);
        }
    }
    cout << "strStableAction=" << strStableAction << endl;
    cout << "strLastAction=" << strLastAction << endl;
    if(_nActionStage == 4)
    {
        Speak("识别到第二个动作");
        Speak(strStableAction);
        nPeopleCount++;
        _nActionStage = 5;
    }
    if (_nActionStage == 5)
    {
        bActionDetect = true;
        //cout << "Test:bActionDetect=" << bActionDetect << endl;
        bOpenpose = false;
        _nActionStage = 1;
    }
}

string RobotAct::ActionThread()
{

    std::string best_action;
    int max_count = 0;
    // unordered_map<std::string,int> action_counts;
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(10.0);

    action_counts.clear();

    while (ros::ok())
    {
        // action_counts[GlobalstrAction]++; // 如果GlobalstrAction一直不更新怎么办 写入回调？
        if ((ros::Time::now() - start_time).toSec() >= timeout.toSec())
        {
            break;
        }
        ros::spinOnce();
    }

    for (const auto &pair : action_counts)
    {
        if (pair.second > max_count)
        {
            best_action = pair.first;
            max_count = pair.second;
        }
    }

    if (max_count > 0)
    {
        cout << "匹配度最高的动作" << best_action << endl;
        std::string Action_best = FindWord(best_action, arKWAction);
        if (Action_best.length() > 0)
        {
            return Action_best;
        }
    }
    else
    {
        cout << "无动作输出" << endl;
        // action_counts.clear();
        return "识别错误";
    }
}

void RobotAct::ActionDetect1()
{
    std::string Actions_recev;

    // 提示用户展示第一个动作
    Speak("动作识别，请在十秒内展示第一个动作");

    // 休眠，给用户准备时间
    std::this_thread::sleep_for(std::chrono::seconds(2)); // 休眠 2 秒，适当调整时间

    // 开始检测第一个动作
    auto actionFuture1 = std::async(std::launch::async, [&]() {
        return ActionThread(); 
    });

    // 等待最多 10 秒以检测动作
    if (actionFuture1.wait_for(std::chrono::seconds(12)) == std::future_status::ready) {
        Actions_recev = actionFuture1.get();
        Speak("识别到第一个动作: " + Actions_recev);
    } else {
        Speak("没有识别到第一个动作。");
    }

    // 清空以准备下一个动作
    Actions_recev.clear();

    // 提示用户展示第二个动作
    Speak("请在十秒内展示下一个动作");

    // 休眠，给用户准备时间
    std::this_thread::sleep_for(std::chrono::seconds(2)); // 休眠 2 秒，适当调整时间

    // 开始检测第二个动作
    auto actionFuture2 = std::async(std::launch::async, [&]() {
        return ActionThread(); 
    });

    // 等待最多 10 秒以检测动作
    if (actionFuture2.wait_for(std::chrono::seconds(12)) == std::future_status::ready) {
        Actions_recev = actionFuture2.get();
        Speak("识别到第二个动作: " + Actions_recev);
    } else {
        Speak("没有识别到第二个动作。");
    }

    // 设置动作检测标志
    bActionDetect = true; // 或根据逻辑设置为 false

}

std::string RobotAct::Obj_trans(const std::string &obj_yoloInput)
{
    auto it = Object_map.find(obj_yoloInput);
    if (it != Object_map.end())
    {
        return it->second; // 返回对应的中文
    }
    else
    {
        return "未知"; // 如果没有找到对应的翻译
    }
}

/// @brief 物品识别
void RobotAct::ObjDetect()
{
    cout << "开始识别物体" << endl;
    Speak("开始识别物体"); // 测试
    ros::spinOnce();
    string strObject;
    strObject = FindWord(strDetect, arKWObject);
    std::string Obj_chinese = Obj_trans(strObject);
    if (strObject.length() > 0)
    {
        Speak("识别到物体" + Obj_chinese);
    }
}

/// @brief 人脸识别
void RobotAct::FaceDetect()
{
    //考虑添加nLastFace 增加准确性
    ROS_INFO("[Face]Recognized Face: %s ",strFace.c_str());
    // if(bPeopleFound == false)
    //     return;
    std::string CurrentFace = "";
    std::vector<std::string> recognizedFaces;
    int StableCount     = 0;
    int StableThreshold = 5;
    while(ros::ok())
    {
        CurrentFace = getFaceFromFacerecog();
        recognizedFaces.push_back(CurrentFace);
        
        if(recognizedFaces.size() > 1 && recognizedFaces.back() == recognizedFaces[recognizedFaces.size() - 2])
        {
            StableCount++;
        }
        else
        {
            StableCount = 0;
        }

        if (StableCount > StableThreshold)
        {
            std::cout << "识别到人脸：" << recognizedFaces.back() << endl;
            break;
        }
        ros::spinOnce();
        sleep(0.2);
    }

    // if (recognizedFaces.back().find("Jack") != std::string::npos)
    // {
    //     Speak("你好，杰克");
    //     bFaceDetect = true;
    //     cout << "[face]bFaceDetect=" << bFaceDetect << endl;
    // }
    // if (recognizedFaces.back().find("Linda") != std::string::npos)
    // {
    //     Speak("你好，琳达");
    //     bFaceDetect = true;
    // }
    // if (recognizedFaces.back().find("Lily") != std::string::npos)
    // {
    //     Speak("你好，莉莉");
    //     bFaceDetect = true;
    // }

    if (recognizedFaces.back().find("gjy") != std::string::npos)
    {
        Speak("你好，郭加悦");
        bFaceDetect = true;
        cout << "[face]bFaceDetect=" << bFaceDetect << endl;
    }
    if (recognizedFaces.back().find("wsx") != std::string::npos)
    {
        Speak("你好，王烁心");
        bFaceDetect = true;
    }
    if (recognizedFaces.back().find("wzy") != std::string::npos)
    {
        Speak("你好，王则与");
        bFaceDetect = true;
    }
    else if (recognizedFaces.back().length() == 0)
    {
        cout << "[Face]未识别到人脸 重新识别...." << endl;
        //bFaceDetect = false;
        FaceDetect();
    }
}

/**********************************************************/
/*                   标志位返回                             */
/**********************************************************/
bool RobotAct::GetFlag_PeopleFound()
{
    return GlobalbPeopleFound;
}

bool RobotAct::GetFlag_ObjectFound()
{
    return GlobalbObjectFound;
}

bool RobotAct::GetResult_FaceRecog()
{
    return bFaceDetect;
}

bool RobotAct::GetResult_ActionDetect()
{
    return bActionDetect;
}

bool RobotAct::GetResult_Grab()
{
    return bGrabDone;
}

bool RobotAct::GetResult_Pass()
{
    return bPassDone;
}

bool RobotAct::GetResult_FixView()
{
    return _bFixView_ok;
}

string RobotAct::getActionFromOpenpose()
{
    return GlobalstrAction;
}

string RobotAct::getFaceFromFacerecog()
{
    return strFace;
}

bool RobotAct::GetResult_bPeopleFoundFailed()
{
    return bPeopleFound_failed;
}

bool RobotAct::GetResult_bObjectFoundFailed()
{
    return bObjectFound_failed;
}
