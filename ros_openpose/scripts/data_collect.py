#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
import math
from cv_bridge import CvBridge
import openpose as op  # 确保你有OpenPose库的Python接口
import sys
import argparse
import message_filters
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from ros_openpose.msg import Frame, Person, BodyPart, Pixel
from sensor_msgs.msg import Image, CameraInfo
import torch
import os
import math
from torch import nn
from torch.autograd import Variable
from std_msgs.msg import String

py_openpose_path = "/usr/local/python/openpose"
picSN = 0
Keypoints = []
#dir_path = os.path.dirname(os.path.realpath(__file__))

try:
    # If you run `make install` (default path is `/usr/local/python` for Ubuntu)
    sys.path.append(py_openpose_path)
    from openpose import pyopenpose as op
except ImportError as e:
    rospy.logerr('OpenPose library could not be found. '
                 'Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
    raise e

class ImageCaptureNode:
    def __init__(self):
        rospy.init_node('image_capture_node', anonymous=True)

        self.bridge = CvBridge()
        self.picSN = 0
        self.picPaths = "0"

        # Subscribe to Kinect2 RGB image topic
        self.image_sub = rospy.Subscriber('/kinect2/hd/image_color', Image, self.image_callback)
        self.image_pub = rospy.Publisher('processed_image', Image, queue_size=10)
        # Initialize OpenPose
        self.op_wrapper = self.initialize_openpose()
        self.save_images = True

    def initialize_openpose(self):
        # 初始化OpenPose相关设置
        params = {
            "model_folder": "/home/bei/openpose/openpose/models",
            "hand": False,
            "face": False,
            "net_resolution" : '320x176',
            "number_people_max": 1,
            "render_threshold" : 0.01 

        }
        op_wrapper = op.WrapperPython()
        op_wrapper.configure(params)
        op_wrapper.start()
        #opWrapper = op.OpenPose(params)
        #opWrapper.start()
        return op_wrapper

    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_pic(cv_image)
        except Exception as e:
            rospy.logerr("Error in image callback: %s", e)

    def process_pic(self, frame):
        try:
            datum = op.Datum()
            datum.cvInputData = frame
            self.op_wrapper.emplaceAndPop(op.VectorDatum([datum]))
        except Exception as e:
            print(f"An error occurred: {e}")

        if datum.poseKeypoints is not None:
            keyPoints = datum.poseKeypoints.tolist()
        if keyPoints is None:
            print("No keypoints detected.")
            #return  # 如果没有检测到关键点，提前返回
        global label
        label = str(input("请输入动作序号:"))
        global picSN
        picSN += 1
        pictureName = str(picSN) + '_' + label + ".jpg"
        self.picPaths = "/home/bei/robot_ws/src/ros_openpose/dataset/ros/pic_background/" + pictureName
        #cv2.imwrite(self.picPaths, datum.cvOutputData)
        cv2.imwrite(self.picPaths, frame)
        dstPicPath = "/home/bei/robot_ws/src/ros_openpose/dataset/ros/marked_pic/p_" + self.picPaths.split('/')[-1]  # 处理后的图片
        cv2.imwrite(dstPicPath, datum.cvOutputData)
        with open("/home/bei/robot_ws/src/ros_openpose/dataset/ros/bone_dataSet.data", "a+") as dataSet:
            dataSet.writelines(
                str(self.pointDistance(keyPoints[0]) + self.pointAngle(keyPoints[0]) + [int(label)]))
            dataSet.write("\n")
        self.publish_processed_image(datum.cvOutputData)

        # 处理图像并保存
        #self.save_pic(datum.cvOutputData)
        #self.publish_processed_image(datum.cvOutputData)

    def save_pic(self, output_frame):
        global picSN
        picSN += 1
        #picture_name = f"{self.picSN}.jpg"
        #pic_paths = f"../dataset/ros/pic_background/{picture_name}"
        pictureName = str(picSN) + '_' + label + ".jpg"

        picPaths = "/home/bei/robot_ws/src/ros_openpose/dataset/ros/pic_background/" + pictureName
        try:
            cv2.imwrite(picPaths, output_frame)
        except Exception as e:
            print(f"An error occurred: {e}")
        rospy.loginfo('Captured image: %s', picPaths)

        # 这里可以处理关键点数据并保存
    def publish_processed_image(self, output_frame):
        # 将OpenCV图像转换为ROS图像消息并发布
        processed_image_msg = self.bridge.cv2_to_imgmsg(output_frame, "bgr8")
        self.image_pub.publish(processed_image_msg)
        rospy.loginfo('Published processed image')
    def pointDistance(self, keyPoint):
        """
        :param keyPoint:
        :return:list
        :distance:
        """
        distance0 = (keyPoint[4][0] - keyPoint[9][0]) ** 2 + \
                    (keyPoint[4][1] - keyPoint[9][1]) ** 2
        distance1 = (keyPoint[7][0] - keyPoint[12][0]) ** 2 + \
                    (keyPoint[7][1] - keyPoint[12][1]) ** 2
        distance2 = (keyPoint[2][0] - keyPoint[4][0]) ** 2 + \
                    (keyPoint[2][1] - keyPoint[4][1]) ** 2
        distance3 = (keyPoint[5][0] - keyPoint[7][0]) ** 2 + \
                    (keyPoint[5][1] - keyPoint[7][1]) ** 2
        distance4 = (keyPoint[0][0] - keyPoint[4][0]) ** 2 + \
                    (keyPoint[0][1] - keyPoint[4][1]) ** 2
        distance5 = (keyPoint[0][0] - keyPoint[7][0]) ** 2 + \
                    (keyPoint[0][1] - keyPoint[7][1]) ** 2
        distance6 = (keyPoint[4][0] - keyPoint[10][0]) ** 2 + \
                    (keyPoint[4][1] - keyPoint[10][1]) ** 2
        distance7 = (keyPoint[7][0] - keyPoint[13][0]) ** 2 + \
                    (keyPoint[7][1] - keyPoint[13][1]) ** 2
        distance8 = (keyPoint[4][0] - keyPoint[7][0]) ** 2 + \
                    (keyPoint[4][1] - keyPoint[7][1]) ** 2
        distance9 = (keyPoint[11][0] - keyPoint[14][0]) ** 2 + \
                    (keyPoint[11][1] - keyPoint[14][1]) ** 2
        distance10 = (keyPoint[10][0] - keyPoint[13][0]
                      ) ** 2 + (keyPoint[10][1] - keyPoint[13][1]) ** 2
        distance11 = (keyPoint[6][0] - keyPoint[10][0]
                      ) ** 2 + (keyPoint[6][1] - keyPoint[10][1]) ** 2
        distance12 = (keyPoint[3][0] - keyPoint[13][0]
                      ) ** 2 + (keyPoint[3][1] - keyPoint[13][1]) ** 2
        distance13 = (keyPoint[4][0] - keyPoint[23][0]
                      ) ** 2 + (keyPoint[4][1] - keyPoint[23][1]) ** 2
        distance14 = (keyPoint[7][0] - keyPoint[20][0]
                      ) ** 2 + (keyPoint[7][1] - keyPoint[20][1]) ** 2

        return [distance0, distance1, distance2, distance3, distance4, distance5, distance6, distance7,
                distance8, distance9, distance10, distance11, distance12, distance13, distance14]

    def pointAngle(self, keyPoint):
        angle0 = self.myAngle(keyPoint[2], keyPoint[3], keyPoint[4])
        angle1 = self.myAngle(keyPoint[5], keyPoint[6], keyPoint[7])
        angle2 = self.myAngle(keyPoint[9], keyPoint[10], keyPoint[11])
        angle3 = self.myAngle(keyPoint[12], keyPoint[13], keyPoint[14])
        angle4 = self.myAngle(keyPoint[3], keyPoint[2], keyPoint[1])
        angle5 = self.myAngle(keyPoint[6], keyPoint[5], keyPoint[1])
        angle6 = self.myAngle(keyPoint[10], keyPoint[8], keyPoint[13])
        angle7 = self.myAngle(keyPoint[7], keyPoint[12], keyPoint[13])
        angle8 = self.myAngle(keyPoint[4], keyPoint[9], keyPoint[10])
        angle9 = self.myAngle(keyPoint[4], keyPoint[0], keyPoint[7])
        angle10 = self.myAngle(keyPoint[4], keyPoint[8], keyPoint[7])
        angle11 = self.myAngle(keyPoint[1], keyPoint[8], keyPoint[13])
        angle12 = self.myAngle(keyPoint[1], keyPoint[8], keyPoint[10])
        angle13 = self.myAngle(keyPoint[4], keyPoint[1], keyPoint[8])
        angle14 = self.myAngle(keyPoint[7], keyPoint[1], keyPoint[8])

        return [angle0, angle1, angle2, angle3, angle4, angle5, angle6, angle7,
                angle8, angle9, angle10, angle11, angle12, angle13, angle14]

    def myAngle(self, A, B, C):
        c = math.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)
        a = math.sqrt((B[0] - C[0]) ** 2 + (B[1] - C[1]) ** 2)
        b = math.sqrt((A[0] - C[0]) ** 2 + (A[1] - C[1]) ** 2)
        if 2 * a * c != 0:
            return (a ** 2 + c ** 2 - b ** 2) / (2 * a * c)
        return 0

if __name__ == "__main__":
    try:
        node = ImageCaptureNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

