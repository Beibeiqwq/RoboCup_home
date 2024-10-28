#include "RobotAct.h"
/**********************************************************/
/*                       定义区                            */
/**********************************************************/

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
static string strToSpeak = "";
static string strKeyWord = "";
bool RobotAct::bActionDetect = false;
int  RobotAct::nPeopleCount = 0;
int  RobotAct::nLitterCount = 0;
int  RobotAct::nPlaceCount  = 1;

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
    n.param<string>("enter", _coord_cmd, "cmd");
    n.param<string>("place1", arKWPlacement[1], "1");
    n.param<string>("place2", arKWPlacement[2], "2");
    n.param<string>("place3", arKWPlacement[3], "3");
    // n.param<string>("place4", arKWPlacement[4], "4");
    // n.param<string>("place5", arKWPlacement[5], "5");
    n.param<string>("dustbin",coord_dustbin,"dustbinA");
    n.param<string>("exit", _coord_exit, "exitA");
    n.param<float> ("PID_Forward", _PID_Forward, 0.0002);
    n.param<float> ("PID_Turn", _PID_Turn, 0.0003);
    /*---------------ROS初始化---------------*/
    sub_yolo         = n.subscribe("/yolo_bbox_2d", 2, &RobotAct::YOLOV5Callback, this);
    sub_pose         = n.subscribe("/Openpose", 10, &RobotAct::OpenPoseCallback, this);
    sub_face         = n.subscribe("/FaceDetect", 10, &RobotAct::FaceRecogCallback, this);
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

    case ACT_CONTACT:
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

    // case ACT_SPEAK:
    //     if (nLastActCode != ACT_SPEAK)
    //     {
    //         printf("[RobotAct] %d - Speak %s\n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
    //         strToSpeak = arAct[nCurActIndex].strTarget;
    //         std_msgs::String rosSpeak;
    //         rosSpeak.data = strToSpeak;
    //         speak_pub.publish(rosSpeak);
    //         strToSpeak = "";
    //         usleep(arAct[nCurActIndex].nDuration * 1000 * 1000);
    //         nCurActIndex++;
    //     }
    //     break;

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
            printf("[RobotAct] %d - Add waypoint %s \n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
            AddNewWaypoint(arAct[nCurActIndex].strTarget);
            nCurActIndex++;
        }
        break;

    case ACT_FIND_PERSON:
        if (nLastActCode != ACT_FIND_PERSON)
        {
            if (!bPeopleFound)
            {
                SetSpeed(0, 0, 0.2); //方案一
                //方案二 遍历房间内航点
                nCurActIndex++;
            }
        }
        break;

    case ACT_FIND_OBJ:
        if (nLastActCode != ACT_FIND_OBJ)
        {
            if (!bObjectFound)
            {
                Speak("找不到物品");
                cout << "!OBJECT_FOUND!!!!!" << endl;
                //SetSpeed(0, 0, 0.2);//方案一
                //方案二 遍历房间内航点
                //nCurActIndex++;
            }
            else
            {
                ObjDetect();
                nCurActIndex++;
            }
        }
        break;

    case ACT_ACTION_DETECT:
        if (nLastActCode != ACT_ACTION_DETECT)
        {
            if (bPeopleFound == true)
            {
                FaceDetect();
                // if(bFaceDetect == true)
                // {
                //     bOpenpose = true;
                //     ActionDetect();
                //     nCurActIndex++;
                // }
                sleep(1);
                ActionDetect();
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
            string Peoplename = FindWord(box_object.name, strPerson);
            //Kinect2 QHD发布的图像 像素为960*540 Kinect2 HD发布的图像 像素为1920*1080
            if (Peoplename.length() > 0)
            {
                //cout << "进入if Peoplename.length()>0" << endl;
                bPeopleFound = true;
                nYoloPeople = i;
                _nImgHeight = box_object.top - box_object.bottom;
                _nImgWidth = box_object.right - box_object.left;
                _nTargetX = 1024;
                _nTargetY = 540;
            }
            else
            {
                strDetect = msg.name[i];
                bPeopleFound = false;
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
    cout << "[OpenPoseCB]接收到OpenPose数据" << endl;
    string strAction;
    string strOpenpose = msg->data;
    if (bOpenpose == true) 
    {
        strAction = FindWord(strOpenpose, arKWAction);
        if (strAction.length() > 0)
        {
            GlobalstrAction = strAction;
        }
    }
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
    //start = ros::Time::now();

    cout << "进入语音回调函数 bKeyVoice==" << bKeyVoice << endl;
    if(bKeyVoice == false)
        return false;
    else
    {
        // printf("识别到: %s\n", req.data.c_str());
        // std::string voice_txt = req.data;
        Speak("你好");
        while(1)
        {
            printf("识别到: %s\n", req.data.c_str());
            std::string voice_txt = req.data;
            if (voice_txt.find("水") != std::string::npos)
            {
                Speak("你要的是水");
                bFinishVoice = true;   
                break;    
            }
            if (voice_txt.find("薯片") != std::string::npos)
            {
                Speak("你要的是薯片");
                bFinishVoice = true;
                break;
            }
            if (voice_txt.find("洗发水") != std::string::npos)
            {
                Speak("你要的是洗发水");
                bFinishVoice = true;
                break;
            }
            if (voice_txt.find("可乐") != std::string::npos)
            {
                Speak("你要的是可乐");
                bFinishVoice = true;
                break;
            }
            if (voice_txt.find("面包") != std::string::npos)
            {
                Speak("你要的是面包");
                bFinishVoice = true;
                break;
            }
            if (voice_txt.find("饼干") != std::string::npos)
            {
                Speak("你要的是饼干");
                bFinishVoice = true;
                break;
            }
            if (voice_txt.find("乐事薯片") != std::string::npos)
            {
                Speak("你要的是乐事薯片");
                bFinishVoice = true;
                break;
            }
            if (voice_txt.find("曲奇") != std::string::npos)
            {
                Speak("你要的是曲奇");
                bFinishVoice = true;
                break;
            }
            if (voice_txt.find("洗洁精") != std::string::npos)
            {
                Speak("你要的是洗洁精");
                bFinishVoice = true;
                break;
            }
            if (voice_txt.find("芬达") != std::string::npos)
            {
                Speak("你要的是芬达");
                bFinishVoice = true;
                break;
            }
            if (voice_txt.find("洗手液") != std::string::npos)
            {
                Speak("你要的是洗手液");
                bFinishVoice = true;
                break;
            }
            else
            {
                Speak("请重新告诉我你要的物品");
                bFinishVoice = false;
                //voice_txt = "";
                // voice_txt = req.data;
                continue;
            }
        }
        bKeyVoice = false;
        resp.success = true;
        return resp.success;
    }

}

/**********************************************************/
/*                   程序功能区                             */
/**********************************************************/

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
    // cout << "Place4:" << arKWPlacement[4] << endl;
    cout << "PID_ForWard:" << _PID_Forward << endl;
    cout << "PID_Turn:" << _PID_Turn << endl;
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

/// @brief 递给开关
/// @param inActive 
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
        ROS_ERROR("[RobotAct]启动服务失败");
    }
}

/// @brief 机械臂抬起
void RobotAct::Raise_arm()
{
    cout << "正在抬起手臂...." << endl;
}

/// @brief 机械臂抓取
void RobotAct::Grab_arm()
{
    cout << "正在抓取..." << endl;
}

/**********************************************************/
/*                   机器人任务区                           */
/**********************************************************/

/// @brief 动作识别
void RobotAct::ActionDetect()
{
    cout << "[ActionDetect]动作识别开始...." << endl;
    Speak("动作识别开始");
    sleep(2);
    GlobalstrAction = "站立";

    if (_nActionStage == 1)
    {
        Speak("请开始你的第一个动作");
        sleep(2);
        Speak("识别到第一个动作");
        Speak(GlobalstrAction);
        _nActionStage = 2;
    }
    if(_nActionStage == 2)
    {
        Speak("你可以展示下一个动作了");
        sleep(2);
        _nActionStage = 3;
    }
    if(_nActionStage == 3)
    {
        Speak("识别到第二个动作");
        Speak(GlobalstrAction);
        //bActionDetect = true;
        nPeopleCount++;
        _nActionStage = 4;
    }
    if (_nActionStage == 4)
    {
        bActionDetect = true;
        cout << "Test:bActionDetect=" << bActionDetect << endl;
        bOpenpose = false;
        _nActionStage = 1;
    }
}

/// @brief 物品识别
void RobotAct::ObjDetect()
{
    cout << "开始识别物体" << endl;
    Speak("开始识别物体"); // 测试
    string strObject;
    strObject = FindWord(strDetect, arKWObject);
    if (strObject.length() > 0)
    {
        Speak("识别到物体" + strObject);
    }
}

/// @brief 人脸识别
void RobotAct::FaceDetect()
{
    //考虑添加nLastFace 增加准确性
    ROS_INFO("[Face]Recognized Face: %s ",strFace.c_str());
    // if(bPeopleFound == false)
    //     return;
    if (strFace.find("gjy") != std::string::npos)
    {
        Speak("你好，郭嘉悦");
        bFaceDetect = true;
        cout << "[face]bFaceDetect=" << bFaceDetect << endl;
    }
    if (strFace.find("lwj") != std::string::npos)
    {
        Speak("你好，林文俊");
        bFaceDetect = true;
        //return;
    }
    if (strFace.find("wsx") != std::string::npos)
    {
        Speak("你好，王烁心");
        bFaceDetect = true;
    }
    if (strFace.find("wzy") != std::string::npos)
    {
        Speak("你好，王则与");
        bFaceDetect = true;
    }
    else if (strFace.length() == 0)
    {
        cout << "【Face】进入Else" << endl;
        //bFaceDetect = false;
        FaceDetect();
    }
}

/**********************************************************/
/*                   标志位返回                             */
/**********************************************************/
bool RobotAct::GetFlag_PeopleFound()
{
    return bPeopleFound;
}

bool RobotAct::GetFlag_ObjectFound()
{
    return bObjectFound;
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

