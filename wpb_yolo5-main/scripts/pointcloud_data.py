#!/usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# 三维点云回调函数
def callbackPointcloud(msg):
    # 从点云中提取三维坐标数值
    rospy.loginfo("Header 坐标系为 %s",msg.header.frame_id )
# 主函数
if __name__ == "__main__":
    rospy.init_node("pointcloud_data")
    # 订阅机器人视觉传感器Kinect2的三维点云话题
    pc_sub = rospy.Subscriber("/camera/depth_registered/points",PointCloud2, callbackPointcloud,queue_size=10)
    rospy.spin()