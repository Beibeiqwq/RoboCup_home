#!/usr/bin/env python
 
import rospy
import cv2 
import numpy as np
import face_recognition
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header,String
 
# 创建图像订阅者
class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        # 特征信息文件名
        self.trainData = r"/home/bei/robot_ws/src/ros_main/face_recognition/data/encoding.txt"
        # 准备空的列表来保存人脸编码和对应的人名
        # 这个需要提前完成
        self.known_face_encodings = []
        self.known_face_names = []
        self.num = 0
        # 训练数据
        # self.trainFunction()
 
        # 读取文件中的训练数据
        self.readInfo()
        self.result_pub = rospy.Publisher("/FaceDetect", String, queue_size=5)
        # 创建一个图像发布者对象，用来发布处理之后的数据
        self.image_pub = rospy.Publisher("/imageDeal", Image, queue_size=5)
 
        self.image_raw_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.subscriberPhotoCallbackUpdate)
 
    def trainFunction(self):
        for person in os.listdir(self.train_dir):
            pix_dir = os.path.join(self.train_dir, person)
            # print(pix_dir)
        #     # 遍历每个人的文件夹中的每一张照片
            for item in os.listdir(pix_dir):
                if item.endswith("jpg") or item.endswith("png"):
                    image_path = os.path.join(pix_dir, item)
                    print(image_path)
#             # 加载图片
                    image = face_recognition.load_image_file(image_path)
#             # 尝试提取一张脸的特征编码
                    face_encoding = face_recognition.face_encodings(image)
                    if face_encoding:
                        self.known_face_encodings.append(face_encoding[0])
                        self.known_face_names.append(person)
 
    def readInfo(self):
        file = open(self.trainData, "r")
        data_raw = file.readlines()
 
        for i in range(len(data_raw)):
            tmp = data_raw[i].strip()
            tmp = tmp.split(" ")
            tmp2 = []
            for j in range(len(tmp) - 1):
                tmp2.append(float(tmp[j]))
                self.known_face_names.append(tmp[len(tmp) - 1])
                self.known_face_encodings.append(tmp2)
        self.known_face_encodings = np.array(self.known_face_encodings)
    
    def subscriberPhotoCallbackUpdate(self, data):
        # 这个函数是用来接受摄像头的数据的
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        #cv2.imshow("hahaha", image_raw)
        #cv2.waitKey(100)
        face_locations = face_recognition.face_locations(image)
        face_encodings = face_recognition.face_encodings(image, face_locations)
        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
            self.num = self.num + 1
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            name = ""
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            best_match = np.argmin(face_distances)
            if matches[best_match] and face_distances[best_match] < 0.70:
                name = self.known_face_names[best_match]
                self.result_pub.publish(name)
            cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 2)
            cv2.rectangle(image, (left, bottom - 15), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(image, name, (left + 6, bottom - 6), font, 3.0, (255, 255, 255), 5)
            #cv2.imwrite(r"/home/bei/robot_ws/src/ros_main/face_recognition/" + str(self.num) + ".jpg", image)
            #cv2.waitKey(500)
        # 将OpenCV的图像信息转换为ROS消息类型
        image_message = bridge.cv2_to_imgmsg(image, "bgr8")
        self.image_pub.publish(image_message)
        cv2.waitKey(3)
 
def main():
    # ROS节点初始化
    rospy.init_node('FaceCompare', anonymous=True)
    # 创建ImageConverter实例
    image_converter = ImageConverter()
    # rospy.loginfo("Image converted and published.")
    rospy.spin()
 
if __name__ == "__main__":
    main()