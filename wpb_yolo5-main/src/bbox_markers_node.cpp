#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <wpb_yolo5/BBox3D.h>


static ros::Publisher marker_pub;
static visualization_msgs::Marker line_box;
static visualization_msgs::Marker text_marker;

static float arR[255];
static float arG[255];
static float arB[255];

void DrawText(int inID,std::string inText, std::string inFrame_ID, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = inFrame_ID;
    text_marker.ns = "line_obj";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = inID;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = inScale;
    text_marker.color.r = inR;
    text_marker.color.g = inG;
    text_marker.color.b = inB;
    text_marker.color.a = 1.0;

    text_marker.pose.position.x = inX;
    text_marker.pose.position.y = inY;
    text_marker.pose.position.z = inZ;
    
    text_marker.pose.orientation=tf::createQuaternionMsgFromYaw(0.0);

    text_marker.text = inText;

    marker_pub.publish(text_marker);
}

void DrawBox(int inID, std::string inName, std::string inFrame_ID, float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB)
{
    line_box.header.frame_id = inFrame_ID;
    line_box.ns = "line_box";
    line_box.action = visualization_msgs::Marker::ADD;
    line_box.id = inID;
    line_box.type = visualization_msgs::Marker::LINE_LIST;
    line_box.scale.x = 0.005;
    line_box.color.r = inR;
    line_box.color.g = inG;
    line_box.color.b = inB;
    line_box.color.a = 1.0;

    line_box.pose.orientation=tf::createQuaternionMsgFromYaw(0.0);

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

    DrawText(inID, inName, inFrame_ID, 0.05, (inMinX+inMaxX)/2, (inMinY+inMaxY)/2, inMaxZ+0.05, 1.0, 0, 1.0);
}

void RemoveMarkers()
{
    line_box.action = 3;
    line_box.points.clear();
    marker_pub.publish(line_box);
    text_marker.action = 3;
    marker_pub.publish(text_marker);
}

void InitRGB()
{
    for(int i=0;i<255;i++)
    {
        arR[i] = 0;
        arG[i] = 1.0;
        arB[i] = 0;
    }
}

void CallbackBBox3d(const wpb_yolo5::BBox3D::ConstPtr& msg)
{
    RemoveMarkers();
    int nNum = msg->name.size();
    // ROS_INFO("[CallbackBBox3d] 接收到 %d 个物品三维坐标",nNum);
    for(int i=0;i<nNum;i++)
    {
        // ROS_INFO("%s frame: %s x(%.2f,%.2f) y(%.2f,%.2f) z(%.2f,%.2f)",msg->name[i].c_str(), msg->frame_id[i].c_str(), msg->x_min[i], msg->x_max[i], msg->y_min[i], msg->y_max[i], msg->z_min[i], msg->z_max[i]);
        DrawBox(i, msg->name[i], msg->frame_id[i], msg->x_min[i], msg->x_max[i], msg->y_min[i], msg->y_max[i], msg->z_min[i], msg->z_max[i], 0, 1, 0);
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "bbox_markers_node");
    ROS_INFO("bbox_markers_node start!");

    ros::NodeHandle nh;
    marker_pub = nh.advertise<visualization_msgs::Marker>("/bbox_marker", 30);
    ros::Subscriber bbox_3d_sub = nh.subscribe("/draw_bbox_3d", 10, CallbackBBox3d);
    
    ros::Rate r(10);

    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;

}
