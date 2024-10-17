#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
import os
import torch
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from wpb_yolo5.msg import BBox2D

start = True

# 彩色图像回调函数
def cbImage(msg):
    global start #是否开始yolo转换
    if start == False:
        return

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")#将ros消息格式转换为cv2格式 

    global model
    # Inference
    results = model(cv_image, size=640)  # includes NMS
    results.print()
        #图像推理结果 将结果转换为Pandas DataFrame 
    bboxs = results.pandas().xyxy[0].values
    bbox2d_msg = BBox2D()#实例化2D识别结果
    for bbox in bboxs:
        # rospy.logwarn("识别物品 %s (%.2f , %.2f)" , bbox[-1],bbox[0],bbox[1])
        bbox2d_msg.name.append(bbox[-1]) #识别到的名字
        bbox2d_msg.probability.append(bbox[4])#置信度
        bbox2d_msg.left.append(bbox[0])#x min
        bbox2d_msg.top.append(bbox[1])#y min
        bbox2d_msg.right.append(bbox[2])#x max
        bbox2d_msg.bottom.append(bbox[3])#y max
    global bbox2d_pub
    bbox2d_pub.publish(bbox2d_msg)#发布识别结果
    
    # 弹出窗口显示图片
    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(1)

def cbCommand(msg):#命令回调函数 可以在别的节点中向/yolo_cmd话题中发布消息控制yolo节点
    global start
    rospy.logwarn('Yolo接受到' + msg.data)
    if msg.data == 'yolo start':
        start = True
        rospy.logwarn('Yolo识别开启...')
    if msg.data == 'yolo stop':
        start = False
        rospy.logwarn('Yolo识别停止！')

# 主函数
if __name__ == "__main__":
    rospy.init_node("yolo5_node")
    #改成自己的绝对路径
    weights_path ='/home/bei/robot_ws/src/yolov5_ros/weights/yolov5s.pt'
    yolov5_path = '/home/bei/robot_ws/src/yolov5_ros/yolov5'

    rospy.logwarn('Weights : ' + weights_path)
 
    # 使用Pytorch加载Yolov5模型
    # torch.hub.load会在Yolov5路径下寻找hubconf.py 包含模型加载代码
    model = torch.hub.load(yolov5_path, 'custom', path=weights_path, source='local')
    # 订阅机器人视觉传感器Kinect2的图像话题
    # image_sub = rospy.Subscriber("/kinect2/qhd/image_color_rect",Image,cbImage,queue_size=10)
    image_sub = rospy.Subscriber("/yolo/image_predict",Image,cbImage,queue_size=10)
    #yolo识别状态控制
    command_sub = rospy.Subscriber("/yolo_cmd",String,cbCommand,queue_size=1)
    #2D识别结果发布 
    bbox2d_pub = rospy.Publisher("/yolo_bbox_2d", BBox2D, queue_size=10)
    rospy.spin()