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

using namespace cv;
using namespace std;
class ImageConverter {
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber	image_sub_color;//接收彩色图像
	image_transport::Subscriber	image_sub_depth;//接收深度图像

	ros::Subscriber camera_info_sub_;	//接收深度图像对应的相机参数话题
	ros::Publisher	arm_point_pub_;		//发布一个三维坐标点，可用于可视化

	sensor_msgs::CameraInfo camera_info;
	geometry_msgs::PointStamped	output_point;

	Mat	colorImage;
	Mat	depthImage;
	Point	mousepos	= Point( 0, 0 );        /* mousepoint to be map */

public:
	//获取鼠标的坐标，通过param指针传出到类成员Point mousepos
	static void on_mouse( int event, int x, int y, int flags, void *param )
	{
		switch ( event )
		{
		case CV_EVENT_MOUSEMOVE:                /* move mouse */
		{
			Point &tmppoint = *(cv::Point *) param;
			tmppoint = Point( x, y );
		} break;
		}
	}


	ImageConverter() : it_( nh_ )
	{
		std::string depth_topic,color_topic,info_topic;
    	ros::NodeHandle nh_param("~");
    	nh_param.param<std::string>("depth_topic", depth_topic, "/camera/aligned_depth_to_color/image_raw");
    	nh_param.param<std::string>("color_topic", color_topic, "/camera/color/image_raw");
    	nh_param.param<std::string>("info_topic", info_topic, "/camera/aligned_depth_to_color/camera_info");

    	//topic sub:
		image_sub_depth = it_.subscribe( depth_topic,1, &ImageConverter::imageDepthCb, this );
		image_sub_color = it_.subscribe( color_topic,1, &ImageConverter::imageColorCb, this );
		camera_info_sub_ = nh_.subscribe( info_topic,1, &ImageConverter::cameraInfoCb, this );
    
    	//topic pub:
		arm_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>( "/mouse_point", 10 );

		cv::namedWindow( "colorImage" );
		setMouseCallback( "colorImage", &ImageConverter::on_mouse, (void *) &mousepos );
	}


	~ImageConverter()
	{
		cv::destroyWindow( "colorImage" );
	}


	void cameraInfoCb( const sensor_msgs::CameraInfo &msg )
	{
		camera_info = msg;
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
		
		//先查询对齐的深度图像的深度信息，根据读取的camera info内参矩阵求解对应三维坐标
		float real_z = 0.001 * depthImage.at<u_int16_t>( mousepos.y, mousepos.x );
		float real_x = (mousepos.x - camera_info.K.at( 2 ) ) / camera_info.K.at( 0 ) * real_z;
		float real_y = (mousepos.y - camera_info.K.at( 5 ) ) / camera_info.K.at( 4 ) * real_z;

		char tam[100];
		sprintf( tam, "(%0.2f,%0.2f,%0.2f)", real_x, real_y, real_z );
		putText( colorImage, tam, mousepos, FONT_HERSHEY_SIMPLEX, 0.6, cvScalar( 0, 0, 255 ), 1 );//打印到屏幕上
		circle( colorImage, mousepos, 2, Scalar( 255, 0, 0 ) );
		output_point.header.frame_id	= "/camera_depth_optical_frame";
		output_point.header.stamp	= ros::Time::now();
		output_point.point.x		= real_x;
		output_point.point.y		= real_y;
		output_point.point.z		= real_z;
		arm_point_pub_.publish( output_point );
		cv::imshow( "colorImage", colorImage );
		cv::waitKey( 1 );
	}

};

int main( int argc, char **argv )
{
	setlocale(LC_ALL,"");
	ros::init( argc, argv, "mouse_to_3d" );
	ImageConverter imageconverter;
	
    ros::Rate r(10);

    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
	return(0);
}