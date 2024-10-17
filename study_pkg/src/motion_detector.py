#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

class motionDetector:
    def __init__(self):
        rospy.on_shutdown(self.cleanup);

        # 创建cv_bridge实例化对象
        self.bridge = CvBridge()
        #初始化cv_bridge图像输出的主题发布者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)

        # 设置参数：最小区域、阈值
        self.minArea   = rospy.get_param("~minArea",   500)
        self.threshold = rospy.get_param("~threshold", 25)

        self.firstFrame = None
        self.text = "Unoccupied"

        # 初始化订阅rgb格式图像数据的订阅者，此处图像topic的话题名可以在launch文件中重映射
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)

    def image_callback(self, data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            #将RGB图像转换为CV_BRIDGE图像，data为输入的RGB图像，bgr8为转换后的CV_BRIDGE图像的格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            #np.array()将图像数据cv_bridge复制到frame中，格式为uint8
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

        # cv2.cvtColor()将frame从BGR格式转换为GRAY格式（灰度格式）
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #cv2.GaussianBlur()进行高斯滤波平滑处理，(21,21)为高斯核的大小
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        # 使用两帧图像做比较，检测移动物体的区域
        if self.firstFrame is None:
            self.firstFrame = gray
            return  
        
        #获取两幅灰度图的差值
        frameDelta = cv2.absdiff(self.firstFrame, gray)
        self.firstFrame = gray
        #将差值图中小于阈值的点置为0，大于阈值的点置为255
        thresh = cv2.threshold(frameDelta, self.threshold, 255, cv2.THRESH_BINARY)[1]

        #膨胀操作，即结构元素的方框里如果有白色像素点，则整个方框的像素点都为白色
        #参一为输入图像，参二为结构元素的方框，参三为迭代次数
        thresh = cv2.dilate(thresh, None, iterations=2)
        #边缘检测，即检测出白色区域的边缘，参二为轮廓查找方式，为同一边缘线的像素点只用起点和终点代替
        #binary为二值图，cnts为边缘点的集合（坐标），
        binary, cnts, hierarchy= cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        maxsqu = 0      #每次检测中最大运动物体的面积
        maxC   = None   #获取每次检测中最大运动物体并框住
        for c in cnts:
            # 如果检测到的区域面积小于设置值，则忽略
            if cv2.contourArea(c) < self.minArea:
               continue 

            if cv2.contourArea(c) > maxsqu:  
               maxsqu = cv2.contourArea(c)
               maxC = c

        # 在输出画面上框出识别到的物体
        #x,y分别为起始点的x坐标和y坐标，w为宽，h为高
        (x, y, w, h) = cv2.boundingRect(maxC)
        #cv2.rectangle()画出边缘的方框，框出物体
        #参一为图像，参二为方框左上角坐标，参三为右下角坐标，参四为方框rgb颜色，参五为宽度
        cv2.rectangle(frame, (x, y), (x + w, y + h), (50, 255, 50), 2)
        self.text = "Occupied"

        # 在输出画面上打当前状态和时间戳信息
        cv2.putText(frame, "Status: {}".format(self.text), (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 将识别后的图像转换成ROS消息并发布
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("motion_detector")
        rospy.loginfo("motion_detector node is started...")
        rospy.loginfo("Please subscribe the ROS image.")
        motionDetector()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down motion detector node."
        cv2.destroyAllWindows()

