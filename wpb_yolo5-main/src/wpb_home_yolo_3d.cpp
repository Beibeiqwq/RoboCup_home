#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <wpb_yolo5/BBox2D.h>
#include <wpb_yolo5/BBox3D.h>

using namespace cv;

static Mat image_color; //彩色图像
static Mat image_predict; //预测图像
static ros::Publisher predict_pub; //图像发布话题 "/yolo/image_predict"
static cv_bridge::CvImage img_bridge; //图像转换 ROS消息 --> CV2
static sensor_msgs::Image predict_img_msg;//图像::ROS消息
static bool flag_predicted = false;//图像发送状态


static ros::Publisher pc_pub; //点云发布
static tf::TransformListener *tf_listener; //TF转换 
static ros::Publisher marker_pub;//标记发布
static ros::Publisher coord_pub; //3D bbox
static visualization_msgs::Marker line_box;//线条
static visualization_msgs::Marker text_marker;//文本

void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB);
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB);
void RemoveBoxes();

typedef struct stObjectBox //定义一个结构体容器 来收yolov5传入的数据
{
    std::string name;
    int left;
    int right;
    int top;
    int bottom;
    float probability;
}stObjectBox;

static std::vector<stObjectBox> objects;//物品
static std::vector<stObjectBox>::const_iterator object_iter;//迭代器

void callbackColorImage(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("callbackColorImage");
    if(flag_predicted == false)
        return;
    
    ROS_INFO("[wpb_home_yolo_3d] 发送一帧图像给 yolo5_node");
    predict_pub.publish(msg); 
    //flag_predicted = false;
}

void callbackBbox(const wpb_yolo5::BBox2D &msg)
{
    ROS_INFO("[wpb_home_yolo_3d] 接收到 yolo5_node 识别结果");
    objects.clear();
    int nNum = msg.name.size();
    if(nNum > 0)
    {
        std::vector<stObjectBox> recv_objects;//收到的物品
        stObjectBox box_object; //bbox格式object 存入收到的msg
        for(int i = 0; i < nNum; i++)
        {
            box_object.name = msg.name[i];//识别到的名字
            box_object.left = msg.left[i]; //x_min
            box_object.right = msg.right[i];//x_max
            box_object.top = msg.top[i];//y_min
            box_object.bottom = msg.bottom[i];//y_max
            box_object.probability = msg.probability[i];//置信度
            recv_objects.push_back(box_object); //id   识别物品名字             x_min            y_min
            ROS_INFO("[识别结果-%d] %s (%.2d,%.2d)",i,box_object.name.c_str(),box_object.left,box_object.top);
        }
        objects = recv_objects; //存入object 
    }
}

void callbackPointCloud(const sensor_msgs::PointCloud2 &input)
{
    ROS_INFO("进入点云回调函数");
    std::vector<stObjectBox> arObject = objects;
    if (arObject.size() <= 0) 
    {
        return;
    }
    
     //to footprint
    sensor_msgs::PointCloud2 pc_footprint;
    bool res = tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(5.0)); 
    if(res == false)
    {
        return;
    }
    pcl_ros::transformPointCloud("/base_footprint", input, pc_footprint, *tf_listener);

    //source cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint , cloud_src);
    //ROS_WARN("cloud_src size = %d  width = %d",cloud_src.size(),input.width); 

    wpb_yolo5::BBox3D coord;
    // Draw Boxes
    RemoveBoxes();
    int object_index = 0;
    std::vector<stObjectBox>::const_iterator i;//使用迭代器遍历
    for (object_iter = arObject.begin(); object_iter != arObject.end(); ++object_iter) 
    {
        int rgb_object_x = (object_iter->left  + object_iter->right)/2;
        int rgb_object_y = (object_iter->top + object_iter->bottom)/2;
        int index_pc = rgb_object_y*input.width + rgb_object_x;

        float object_x = cloud_src.points[index_pc].x;
        float object_y = cloud_src.points[index_pc].y;
        float object_z = cloud_src.points[index_pc].z;

        // 扩散寻找有效坐标点
        for(int nc=0;nc<100;nc++)
        {
            int tmp_index = index_pc;
            object_x = cloud_src.points[tmp_index].x;
            if(std::fpclassify(object_x) != FP_NAN)
            {
                object_y = cloud_src.points[tmp_index].y;
                object_z = cloud_src.points[tmp_index].z;
                break;
            }

            tmp_index = index_pc-nc;
            if(tmp_index>=0 && tmp_index < 1920*1080)
            {
                object_x = cloud_src.points[tmp_index].x;
                if(std::fpclassify(object_x) != FP_NAN)
                {
                    object_y = cloud_src.points[tmp_index].y;
                    object_z = cloud_src.points[tmp_index].z;
                    break;
                }
            }

            tmp_index = index_pc+nc;
            if(tmp_index>=0 && tmp_index < 1920*1080)
            {
                object_x = cloud_src.points[tmp_index].x;
                if(std::fpclassify(object_x) != FP_NAN)
                {
                    object_y = cloud_src.points[tmp_index].y;
                    object_z = cloud_src.points[tmp_index].z;
                    break;
                }
            }

            tmp_index = index_pc-1920*nc;
            if(tmp_index>=0 && tmp_index < 1920*1080)
            {
                object_x = cloud_src.points[tmp_index].x;
                if(std::fpclassify(object_x) != FP_NAN)
                {
                    object_y = cloud_src.points[tmp_index].y;
                    object_z = cloud_src.points[tmp_index].z;
                    break;
                }
            }

            tmp_index = index_pc+1920*nc;
            if(tmp_index>=0 && tmp_index < 1920*1080)
            {
                object_x = cloud_src.points[tmp_index].x;
                if(std::fpclassify(object_x) != FP_NAN)
                {
                    object_y = cloud_src.points[tmp_index].y;
                    object_z = cloud_src.points[tmp_index].z;
                    break;
                }
            }
        }

        if(object_x > 0 && object_x < 1.7)
        {
            // 在疑似目标物有效坐标点附近滤波出点云
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_ptr;
            cloud_source_ptr = cloud_src.makeShared(); 
            pcl::PassThrough<pcl::PointXYZRGB> pass;//设置滤波器对象
            pass.setInputCloud (cloud_source_ptr);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (object_z-0.1, object_z+0.3);
            pass.filter (*cloud_source_ptr);
            pass.setFilterFieldName ("x");
            pass.setFilterLimits (object_x-0.2, object_x+0.1);
            pass.filter (*cloud_source_ptr);
            pass.setFilterFieldName ("y");
            pass.setFilterLimits (object_y-0.2, object_y+0.1);
            pass.filter (*cloud_source_ptr);

            // 检测平面
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
            segmentation.setInputCloud(cloud_source_ptr);
            segmentation.setModelType(pcl::SACMODEL_PLANE);
            segmentation.setMethodType(pcl::SAC_RANSAC);
            segmentation.setDistanceThreshold(0.005);
            segmentation.setOptimizeCoefficients(true);
            Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);
            segmentation.setAxis(axis);
            segmentation.setEpsAngle(  10.0f * (M_PI/180.0f) );
            pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
            segmentation.segment(*planeIndices, *coefficients);
            
            // 获取平面高度
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud (cloud_source_ptr);
            extract.setIndices (planeIndices);
            extract.setNegative (false);
            extract.filter (*plane);
            float plane_height = plane->points[0].z;
            int points_num = plane->points.size();
            for(int i=0;i<points_num;i++)
            {
                if(plane->points[i].z > plane_height)
                {
                    plane_height = plane->points[i].z;
                }
            }
            ROS_INFO("[obj_ind= %d] - plane: %d points. height =%.2f" ,object_index, plane->width * plane->height,plane_height);

            // 剔除平面,只留物品
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr tbjects(new pcl::PointCloud<pcl::PointXYZRGB>);
            pass.setInputCloud (cloud_source_ptr);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (plane_height-0.1, plane_height+0.1);
            pass.filter (*tbjects);
            
            // 找平面上的物品
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            std::vector<pcl::PointIndices> cluster_indices;
            ec.setClusterTolerance (0.1);
            ec.setMinClusterSize (1);
            ec.setMaxClusterSize (5000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (tbjects);
            ec.extract (cluster_indices);

            // 找出点数最多的物品
            pcl::ExtractIndices<pcl::PointXYZRGB> extract_object_indices;
            int obj_num = cluster_indices.size();
            if(obj_num == 0)
                continue;
            int max_index = 0;
            int max_size = 0;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            extract_object_indices.setInputCloud(tbjects);
            extract_object_indices.setIndices(boost::make_shared<const pcl::PointIndices>(cluster_indices[max_index]));
            extract_object_indices.filter(*object_cloud);
            max_size = object_cloud->points.size();
            for(int i = 1; i<obj_num; ++i)
            {
                extract_object_indices.setIndices(boost::make_shared<const pcl::PointIndices>(cluster_indices[i]));
                extract_object_indices.filter(*object_cloud);
                if(object_cloud->points.size() > max_size)
                {
                    max_index = i;
                    max_size = object_cloud->points.size();
                }
            }

            // 对点数最多的物品进行坐标值统计
            float xMax,xMin,yMax,yMin,zMax,zMin;
            extract_object_indices.setIndices(boost::make_shared<const pcl::PointIndices>(cluster_indices[max_index]));
            extract_object_indices.filter(*object_cloud);
            bool bFirstPoint = true;
            for (int j = 0; j < object_cloud->points.size(); j++) 
            {
                pcl::PointXYZRGB p = object_cloud->points[j];
                if(bFirstPoint == true)
                {
                    xMax = xMin = p.x;
                    yMax = yMin = p.y;
                    zMax = zMin = p.z;
                    bFirstPoint = false;
                }

                if(p.x < xMin) { xMin = p.x;}
                if(p.x > xMax) { xMax = p.x;}
                if(p.y < yMin) { yMin = p.y;}
                if(p.y > yMax) { yMax = p.y;}
                if(p.z < zMin) { zMin = p.z;}
                if(p.z > zMax) { zMax = p.z;}
            }
            object_x = (xMax + xMin)/2;
            object_y = (yMax + yMin)/2;
            object_z = (zMax + zMin)/2;

            // 将物品结果发布出去
            coord.name.push_back(object_iter->name);
            coord.x_min.push_back(xMin);
            coord.x_max.push_back(xMax);
            coord.y_min.push_back(yMin);
            coord.y_max.push_back(yMax);
            coord.z_min.push_back(zMin);
            coord.z_max.push_back(zMax);


            object_index ++;

            float obj_width = 0.03;
            float obj_height = 0.1;
            //DrawBox(object_x-obj_width , object_x+obj_width, object_y-obj_width, object_y+obj_width, object_z-obj_height, object_z+obj_height, 0, 1, 0);
            DrawBox(xMin, xMax, yMin, yMax, zMin, zMax, 0, 1, 0);
            DrawText(object_iter->name,0.08, object_x, object_y, object_z + 0.12, 1,0,1);
        }

        ROS_WARN("%s (%d,%d) - (%.2f %.2f %.2f) - %.2f",object_iter->name.c_str(), rgb_object_x,rgb_object_y,object_x,object_y,object_z,object_iter->probability); 
    }
    //marker_pub.publish(line_box);
    coord_pub.publish(coord);
    objects.clear();
}
//传入的数据框出识别对象
void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB)
{
    line_box.header.frame_id = "base_footprint";
    line_box.ns = "line_box";
    line_box.action = visualization_msgs::Marker::ADD;
    line_box.id = 0;
    line_box.type = visualization_msgs::Marker::LINE_LIST;
    line_box.scale.x = 0.005;
    line_box.color.r = inR;
    line_box.color.g = inG;
    line_box.color.b = inB;
    line_box.color.a = 1.0;

    geometry_msgs::Point p;
    p.z = inMinZ;
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

    p.z = inMaxZ;
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);
    marker_pub.publish(line_box);
}

static int nTextNum = 2;
//识别到的文本
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = "base_footprint";
    text_marker.ns = "line_obj";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = nTextNum;
    nTextNum ++;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = inScale;
    text_marker.color.r = inR;
    text_marker.color.g = inG;
    text_marker.color.b = inB;
    text_marker.color.a = 1.0;

    text_marker.pose.position.x = inX;
    text_marker.pose.position.y = inY;
    text_marker.pose.position.z = inZ;
    
    text_marker.pose.orientation=tf::createQuaternionMsgFromYaw(1.0);

    text_marker.text = inText;

    marker_pub.publish(text_marker);
}

void RemoveBoxes() //清除数据
{
    line_box.action = 3; //DELETE_ALL
    line_box.points.clear();//点数据清除
    marker_pub.publish(line_box);
    text_marker.action = 3;//DELETE_ALL
    marker_pub.publish(text_marker);
}

void callbackCmd(const std_msgs::String &msg)
{
    ROS_INFO("[wpb_home_yolo_3d] 接收到 start 指令");
    int nFindIndex = 0;
    nFindIndex = msg.data.find("start");
    if( nFindIndex >= 0 )
    {
        flag_predicted = true;
        ROS_WARN("[yolo_start]");
    }
}


int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "wpb_home_yolo_3d");
    ros::NodeHandle nh_param("~");
    ROS_WARN("[wpb_home_yolo_3d] 启动");

    nh_param.param<bool>("start", flag_predicted, false);

    tf_listener = new tf::TransformListener(); 
    ros::NodeHandle nh;
    //彩色图像订阅
    ros::Subscriber rgb_sub = nh.subscribe("/kinect2/qhd/image_color_rect", 1 , callbackColorImage);
    //点云图像订阅
    //ros::Subscriber pc_sub = nh.subscribe("/kinect2/qhd/points", 1 , callbackPointCloud);
    //命令订阅
    ros::Subscriber cmd_sub = nh.subscribe("/yolo/cmd", 1 , callbackCmd);
    //2D识别结果订阅
    ros::Subscriber yolo_sub = nh.subscribe("/yolo_bbox_2d", 1 , callbackBbox);
    //物品标志发布
    marker_pub = nh.advertise<visualization_msgs::Marker>("/obj_marker", 1);
    //3D坐标发布
    coord_pub = nh.advertise<wpb_yolo5::BBox3D>("/yolo/coord", 10);
    //图像发布->yolov5节点
    predict_pub = nh.advertise<sensor_msgs::Image>("/yolo/image_predict", 1);

    ros::Rate loop_rate(300);
    while( ros::ok())
    {
        ros::spin();
        loop_rate.sleep();
    }

    delete tf_listener; 

    return 0;
}
