#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <wpb_yolo5/BBox2D.h>
#include <wpb_yolo5/BBox3D.h>

using namespace cv;
using namespace std;
class ImageTo3D {
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber	image_sub_color;//接收彩色图像
	image_transport::Subscriber	image_sub_depth;//接收深度图像

	ros::Subscriber camera_info_sub_;	//接收深度图像对应的相机参数话题
	ros::Publisher	bbox_3d_pub_;		//发布三维标注信息
	ros::Subscriber	bbox2d_sub_;			//接收Yolo识别框信息

	sensor_msgs::CameraInfo camera_info;
	wpb_yolo5::BBox3D	bbox_3d_msg;

	Mat	colorImage;
	Mat	depthImage;
	wpb_yolo5::BBox2D recv_bbox2d;

	tf2_ros::Buffer* tfBuffer;
  	tf2_ros::TransformListener* tfListener;
	geometry_msgs::PoseStamped pose_before;
	geometry_msgs::PoseStamped pose_transformed;

public:
	ImageTo3D() : it_( nh_ )
	{
		tfBuffer = new tf2_ros::Buffer();
  		tfListener = new tf2_ros::TransformListener(*tfBuffer);

		std::string depth_topic,color_topic,info_topic;
    	ros::NodeHandle nh_param("~");
    	nh_param.param<std::string>("depth_topic", depth_topic, "/camera/aligned_depth_to_color/image_raw");
    	nh_param.param<std::string>("color_topic", color_topic, "/camera/color/image_raw");
    	nh_param.param<std::string>("info_topic", info_topic, "/camera/aligned_depth_to_color/camera_info");

    	//topic sub:
		image_sub_depth = it_.subscribe( depth_topic,1, &ImageTo3D::imageDepthCb, this );
		image_sub_color = it_.subscribe( color_topic, 1, &ImageTo3D::imageColorCb, this );
		camera_info_sub_ = nh_.subscribe( info_topic, 1, &ImageTo3D::cameraInfoCb, this );
		bbox2d_sub_ =  nh_.subscribe( "/yolo_bbox_2d", 10, &ImageTo3D::bbox2dCb, this );

    	//topic pub:
		bbox_3d_pub_ = nh_.advertise<wpb_yolo5::BBox3D>( "/draw_bbox_3d", 10 );

		cv::namedWindow( "colorImage" );
	}


	~ImageTo3D()
	{
		cv::destroyWindow( "colorImage" );
		delete tfListener;
		delete tfBuffer;
	}


	void cameraInfoCb( const sensor_msgs::CameraInfo &msg )
	{
		camera_info = msg;
	}

	void bbox2dCb( const wpb_yolo5::BBox2D &msg )
	{
		recv_bbox2d = msg;
		resetBBox3D();
		// 枚举二维标注框
		int obj_num = msg.name.size();
		for(int i=0 ;i<obj_num; i++)
		{
			// 计算物品在深度图的中心坐标
			int x_center = ((int)msg.left[i] + (int)msg.right[i])/2;
			int y_center = ((int)msg.top[i] + (int)msg.bottom[i])/2;
			float z = 0.001 * depthImage.at<u_int16_t>(y_center , x_center);
			float x = (x_center - camera_info.K.at(2)) / camera_info.K.at(0) * z;
			float y = (y_center - camera_info.K.at(5)) / camera_info.K.at(4) * z;
			float z_scale = z;
			// 转换坐标系到 base_footprint
			pose_before.header.frame_id = "camera_color_optical_frame";
			pose_before.header.stamp = ros::Time(0.0);
			pose_before.pose.position.x = x;
			pose_before.pose.position.y = y;
			pose_before.pose.position.z = z;
			try
			{ 
				tfBuffer->transform(pose_before, pose_transformed, "base_footprint");
				x = pose_transformed.pose.position.x;
				y = pose_transformed.pose.position.y;
				z = pose_transformed.pose.position.z;
			}
			catch(const std::exception& e)
			{
				ROS_INFO("tfBuffer 转换失败！");
			}
			// 计算三维框的边长
			float z_side = fabs((fabs((float)(msg.bottom[i] - msg.top[i])) ) / camera_info.K.at(0)) * z_scale *0.5;
			float x_side = fabs((fabs((float)(msg.right[i] - msg.left[i])) ) / camera_info.K.at(4)) * z_scale *0.5;
			float y_side = x_side;
			// 添加三维标注框
			addBBox3D(msg.name[i], "base_footprint", x-x_side, x+x_side, y-y_side, y+y_side, z-z_side, z+z_side);
			// 在彩色图上标注二维框
			line(colorImage, Point((int)msg.left[i] , (int)msg.top[i]) , Point((int)msg.right[i] , (int)msg.top[i]) ,cvScalar( 0, 255, 0 ),2);
			line(colorImage, Point((int)msg.left[i] , (int)msg.bottom[i]) , Point((int)msg.right[i] , (int)msg.bottom[i]) ,cvScalar( 0, 255, 0 ),2);
			line(colorImage, Point((int)msg.left[i] , (int)msg.top[i]), Point((int)msg.left[i] , (int)msg.bottom[i]) ,cvScalar( 0, 255, 0 ),2);
			line(colorImage, Point((int)msg.right[i] , (int)msg.top[i]), Point((int)msg.right[i] , (int)msg.bottom[i]) ,cvScalar( 0, 255, 0 ),2);

			char text[100];
			// sprintf( text, "(%0.2f,%0.2f,%0.2f)", x, y, z );
			// putText( colorImage, text, Point(y_center, x_center), FONT_HERSHEY_SIMPLEX, 0.6, cvScalar( 0, 0, 255 ), 1 );//打印到图像上
			sprintf( text, "%s", msg.name[i].c_str() );
			putText( colorImage, text, Point(x_center , y_center-80), FONT_HERSHEY_SIMPLEX, 0.6, cvScalar( 255, 0, 255 ), 1);//打印到图像上
			// circle( colorImage, Point(y_center, x_center), 2, Scalar( 255, 0, 0 ) );
			ROS_INFO("物体 %s 坐标 ( %d , %d )",msg.name[i].c_str(),x_center,y_center);
			ROS_WARN("物体 %s 三维坐标 ( %.2f , %.2f , %.2f )",msg.name[i].c_str(),x,y,z);
		}
		// 发送三维标注框
		pubBBox3D();

		cv::imshow( "colorImage", colorImage );
		cv::waitKey( 1 );
	}


	void imageDepthCb( const sensor_msgs::ImageConstPtr &msg )
	{
		cv_bridge::CvImagePtr cv_ptr;

		try {
			cv_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::TYPE_16UC1 );
			depthImage = cv_ptr->image;
		} catch ( cv_bridge::Exception &e ) {
			ROS_ERROR( "cv_bridge exception: %s", e.what() );
			return;
		}
	}


	void imageColorCb( const sensor_msgs::ImageConstPtr &msg )
	{
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr		= cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
			colorImage	= cv_ptr->image;
		} catch ( cv_bridge::Exception &e ) {
			ROS_ERROR( "cv_bridge exception: %s", e.what() );
			return;
		}
	}

	void resetBBox3D()
	{
		bbox_3d_msg.name.clear();
		bbox_3d_msg.frame_id.clear();
		bbox_3d_msg.x_min.clear();
		bbox_3d_msg.x_max.clear();
		bbox_3d_msg.y_min.clear();
		bbox_3d_msg.y_max.clear();
		bbox_3d_msg.z_min.clear();
		bbox_3d_msg.z_max.clear();
	}

	void addBBox3D(std::string inName, std::string inFrame_ID, float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ)
	{
		bbox_3d_msg.name.push_back(inName);
		bbox_3d_msg.frame_id.push_back(inFrame_ID);
		bbox_3d_msg.x_min.push_back(inMinX);
		bbox_3d_msg.x_max.push_back(inMaxX);
		bbox_3d_msg.y_min.push_back(inMinY);
		bbox_3d_msg.y_max.push_back(inMaxY);
		bbox_3d_msg.z_min.push_back(inMinZ);
		bbox_3d_msg.z_max.push_back(inMaxZ);
	}

	void pubBBox3D()
	{
		bbox_3d_pub_.publish(bbox_3d_msg);
	}
};

int main( int argc, char **argv )
{
	setlocale(LC_ALL,"");
	ros::init( argc, argv, "rgb_to_3d" );
	ImageTo3D image_to_3d;
	
    ros::Rate r(10);

    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
	return(0);
}