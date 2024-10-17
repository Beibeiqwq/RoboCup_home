#include "RobotAct.h"
static string strToSpeak = "";
static string strKeyWord = "";

/*---------------别名定义区---------------*/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


RobotAct::RobotAct()
{
    nCurActIndex = 0;
    nCurActCode = -1;
    strListen = "";
    bGrabDone = false;
    bPassDone = false;
}

RobotAct::~RobotAct()
{
}

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
    n.param<string>("exit", _coord_exit, "exitA");
    n.param<float>("PID_Forward", _PID_Forward, 0.002);
    n.param<float>("PID_Turn", _PID_Turn, 0.003);
    /*---------------ROS初始化---------------*/
    sub_yolo = n.subscribe("/yolo_bbox_2d", 10, &RobotAct::YOLOV5CB, this);
    sub_pose = n.subscribe("/Openpose", 10, &RobotAct::OpenPoseCB, this);
    grab_result_sub = n.subscribe<std_msgs::String>("/wpb_home/grab_result", 30, &RobotAct::GrabResultCallback, this);
    pass_result_sub = n.subscribe<std_msgs::String>("/wpb_home/pass_result", 30, &RobotAct::PassResultCallback, this);
    client_speak = n.serviceClient<robot_voice::StringToVoice>("str2voice");
    cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    speak_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20);
    speed_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    yolo_pub = n.advertise<std_msgs::String>("/yolov5/cmd", 20);
    behaviors_pub = n.advertise<std_msgs::String>("/wpb_home/behaviors", 30);
    add_waypoint_pub = n.advertise<waterplus_map_tools::Waypoint>("/waterplus/add_waypoint", 1);
    /*---------------主程序区域---------------*/
    cout << "[Init]请检查程序参数...." << endl;
    Parameter_Check();
    cout << "[Init]键入任意数字开始.... 按CTRL+Z退出" << endl;
    cin >> _check_flag;
}

// 抓取开关
void RobotAct::GrabSwitch(bool inActive)
{
    std_msgs::String behavior_msg;
    if (inActive == true)
    {
        behavior_msg.data = "grab start";
        behaviors_pub.publish(behavior_msg);
    }
    else
    {
        behavior_msg.data = "grab stop";
        behaviors_pub.publish(behavior_msg);
    }
}
// 递给开关
void RobotAct::PassSwitch(bool inActive)
{
    std_msgs::String behavior_msg;
    if (inActive == true)
    {
        behavior_msg.data = "pass start";
        behaviors_pub.publish(behavior_msg);
    }
    else
    {
        behavior_msg.data = "pass stop";
        behaviors_pub.publish(behavior_msg);
    }
}
// 新航点添加
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

static int nLastActCode = -1;
static geometry_msgs::Twist vel_cmd;
// 主状态机
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
             printf("[RobotAct] %d - Find %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            Goto(StrGoto); 
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
            printf("[RobotAct] %d - Pass %s done!\n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
            PassSwitch(false);
            nCurActIndex++;
        }
        break;

    case ACT_SPEAK:
        if (nLastActCode != ACT_SPEAK)
        {
            printf("[RobotAct] %d - Speak %s\n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
            strToSpeak = arAct[nCurActIndex].strTarget;
            std_msgs::String rosSpeak;
            rosSpeak.data = strToSpeak;
            speak_pub.publish(rosSpeak);
            strToSpeak = "";
            usleep(arAct[nCurActIndex].nDuration * 1000 * 1000);
            nCurActIndex++;
        }
        break;

    // case ACT_LISTEN:
    //     if (nLastActCode != ACT_LISTEN)
    //     {
    //         printf("[RobotAct] %d - Listen %s\n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
    //         strListen = "";
    //         strKeyWord = arAct[nCurActIndex].strTarget;
    //         int nDur = arAct[nCurActIndex].nDuration;
    //         if (nDur < 3)
    //         {
    //             nDur = 3;
    //         }
    //         // 开始语音识别
    //         srvIAT.request.active = true;
    //         srvIAT.request.duration = nDur;
    //         clientIAT.call(srvIAT);
    //     }
    //     nKeyWord = strListen.find(strKeyWord);
    //     if (nKeyWord >= 0)
    //     {
    //         // 识别完毕,关闭语音识别
    //         srvIAT.request.active = false;
    //         clientIAT.call(srvIAT);
    //         nCurActIndex++;
    //     }
    //     break;

    case ACT_MOVE:
        printf("[RobotAct] %d - Move ( %.2f , %.2f ) - %.2f\n", nCurActIndex, arAct[nCurActIndex].fLinear_x, arAct[nCurActIndex].fLinear_y, arAct[nCurActIndex].fAngular_z);
        vel_cmd.linear.x = arAct[nCurActIndex].fLinear_x;
        vel_cmd.linear.y = arAct[nCurActIndex].fLinear_y;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = arAct[nCurActIndex].fAngular_z;
        speed_pub.publish(vel_cmd);

        usleep(arAct[nCurActIndex].nDuration * 1000 * 1000);
        nCurActIndex++;
        break;

    case ACT_ADD_WAYPOINT:
        if (nLastActCode != ACT_ADD_WAYPOINT)
        {
            printf("[ActMgr] %d - Add waypoint %s \n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
            AddNewWaypoint(arAct[nCurActIndex].strTarget);
            nCurActIndex++;
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
    arAct.clear();
}

string RobotAct::GetToSpeak()
{
    string strRet = strToSpeak;
    strToSpeak = "";
    return strRet;
}

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
    if (inAct->nAct == ACT_MOVE)
    {
        ActText = "移动 ( ";
        std::ostringstream stringStream;
        stringStream << inAct->fLinear_x << " , " << inAct->fLinear_y << " ) - " << inAct->fAngular_z;
        std::string retStr = stringStream.str();
        ActText += retStr;
    }
    if (inAct->nAct == ACT_ADD_WAYPOINT)
    {
        ActText = "添加航点 ";
        std::ostringstream stringStream;
        // stringStream << inAct->fFollowDist;
        std::string retStr = stringStream.str();
        ActText += retStr;
    }
    return ActText;
}

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

void RobotAct::GrabResultCallback(const std_msgs::String::ConstPtr &res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if (nFindIndex >= 0)
    {
        bGrabDone = true;
    }
}

void RobotAct::PassResultCallback(const std_msgs::String::ConstPtr &res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if (nFindIndex >= 0)
    {
        bPassDone = true;
    }
}

/// @brief 机器人速度发布
/// @param inVx 向前/后移动速度
/// @param inVy 向左/右移动速度
/// @param inTz 旋转角速度
void RobotAct::SetSpeed(float inVx, float inVy, float inTz)
{
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = VelFixed(inVx, _vel_max);
    vel_cmd.linear.y = VelFixed(inVy, _vel_max);
    vel_cmd.angular.z = VelFixed(inTz, _vel_max);
    cout << "设置机器人速度为" << "X:" << inVx << "Y:" << inVy << "Z:" << inTz << endl;
    speed_pub.publish(vel_cmd);
}

/// @brief YOLO回调
/// @param msg
void RobotAct::YOLOV5CB(const wpb_yolo5::BBox2D &msg)
{
    cout << "[YOLOV5CB]:接收到Yolov5数据" << endl;
    YOLO_BBOX.clear();
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
            box_object.right = msg.right[i];             // x_max
            box_object.top = msg.top[i];                 // y_min
            box_object.bottom = msg.bottom[i];           // y_max
            box_object.probability = msg.probability[i]; // 置信度
            recv_BBOX.push_back(box_object);
            std::string strDetect = msg.name[i];
            string Peoplename = FindWord(box_object.name, arKWPerson);
            if (Peoplename.length() > 0)
            {
                nYoloPeople = i;
                _nImgHeight = box_object.top - box_object.bottom;
                _nImgWidth = box_object.right - box_object.left;
                _nTargetX = 128;
                _nTargetY = 128;
            }
        }
        YOLO_BBOX = recv_BBOX; // 存入object
    }

    ///@brief 位姿修正
    if (bFixView == true)
    {
        cout << "[FixView]位姿修正开始...." << endl;
        _fVelForward = _fVelTurn = 0;
        if (nNum != 0 | nNum != NULL)
        {
            if (YOLO_BBOX[nYoloPeople].left != NULL && YOLO_BBOX[nYoloPeople].top != NULL)
            {
                cout << "标定位置信息为:" << "x:" << YOLO_BBOX[nYoloPeople].left << "y:" << YOLO_BBOX[nYoloPeople].top << endl;
                if (YOLO_BBOX[nYoloPeople].left < 128)
                {
                    // fVelForward = (nImgHeight / 2 - nTargetY) * PID_Forward;
                    _fVelTurn = (_nImgWidth / 2 - _nTargetY) * _PID_Turn;
                }
                else if (YOLO_BBOX[nYoloPeople].left > 128)
                {
                    _fVelTurn = (_nImgWidth / 2 - _nTargetY) * _PID_Turn;
                }
                else if (YOLO_BBOX[nYoloPeople].top < 128)
                {
                    _fVelForward = (_nImgHeight / 2 - _nTargetY) * _PID_Forward;
                }
                else if (YOLO_BBOX[nYoloPeople].top > 128)
                {
                    _fVelForward = (_nImgHeight / 2 - _nTargetY) * _PID_Forward;
                }
                else
                {
                    _fVelForward = _fVelForward = 0;
                }
            }
        }
        SetSpeed(VelFixed(_fVelForward, _vel_max), 0, VelFixed(_fVelTurn, _vel_max));
        bFixView_ok = true;
        bFixView = false;
    }
}

/// @brief OpenPose回调
void RobotAct::OpenPoseCB(const std_msgs::String::ConstPtr &msg)
{
    cout << "[OpenPoseCB]接收到OpenPose数据" << endl;
    string sAction;
    string strOpenpose = msg->data;
    if (bOpenpose == true) // 写为开关模式？
    {
        sAction = FindWord(strOpenpose, arKWAction);
        if (sAction.length() > 0)
        {
            strDetect = sAction;
        }
    }
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

/// @brief 出门
void RobotAct::Exit()
{
    cout << "[RobotAct]正在前往出门地点...." << endl;
    Goto(_coord_exit);
}

/// @brief 进门
void RobotAct::Enter()
{
    cout << "[RobotAct]正在前往进门地点...." << endl;
    Goto(_coord_cmd);
}

/// @brief 程序参数打印
void RobotAct::Parameter_Check()
{
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>> Parameter_Check <<<<<<<<<<<<<<<<<<<<<<<" << endl;
    cout << "Yaml Name:" << _name_yaml << endl;
    cout << "Enter Coord:" << _coord_cmd << endl;
    cout << "Exit  Coord:" << _coord_exit << endl;
    cout << "Place1:" << arKWPlacement[1] << endl;
    cout << "Place2:" << arKWPlacement[2] << endl;
    cout << "Place3:" << arKWPlacement[3] << endl;
    cout << "Place4:" << arKWPlacement[4] << endl;
    cout << "PID_ForWard:" << _PID_Forward << endl;
    cout << "PID_Turn:" << _PID_Turn << endl;
    cout << ">>>>>>>>>>>>>>>>>>>> Please check the parameter <<<<<<<<<<<<<<<<<<" << endl;
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
