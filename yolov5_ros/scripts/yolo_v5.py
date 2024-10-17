#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov5_ros.msg import BoundingBox, BoundingBoxes

class Yolo_Dect:
    def __init__(self):
        # 指定Yolov5路径 value="/home/bei/robot_ws/src/yolov5_ros/yolov5"
        yolov5_path = rospy.get_param('/yolov5_path', '')
        # 指定权重文件路径 value="/home/bei/robot_ws/src/yolov5_ros/weights/yolov5s.pt"
        weight_path = rospy.get_param('~weight_path', '')
        #接受图像的主题/配置文件为 value="/kinect2/hd/image_color"
        image_topic = rospy.get_param(
            '~image_topic', '/usb_cam/image_raw')
        #发布话题 value="/yolov5/BoundingBoxes"
        pub_topic = rospy.get_param('~pub_topic', '/yolov5/BoundingBoxes')
        #摄像头参数
        self.camera_frame = rospy.get_param('~camera_frame', '')
        #Yolo参数
        conf = rospy.get_param('~conf', '0.5')
        # 使用Pytorch加载Yolov5模型
        # torch.hub.load会在Yolov5路径下寻找hubconf.py 包含模型加载代码
        self.model = torch.hub.load(yolov5_path, 'custom',
                                    path=weight_path, source='local')
        # 默认使用cuda 没有cuda在配置文件中更改为True
        if (rospy.get_param('/use_cpu', 'false')):
            self.model.cpu()
        else:
            self.model.cuda()
        #conf: 这个参数是物体检测中的置信度阈值。
        self.model.conf = conf
        #彩色图像和深度图像（？）
        self.color_image = Image()
        self.depth_image = Image()
        #初始化将图像状态置为False
        self.getImageStatus = False

        # 框选颜色
        self.classes_colors = {}

        # 图像订阅
        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback,
                                          queue_size=1, buff_size=52428800)

        # 发布者BoundingBoxes包括

        # float64 probability 置信度
        # int64 xmin
        # int64 ymin
        # int64 xmax
        # int64 ymax
        # int16 num
        # string Class  类名

        #图像信息发布 pub_topic = rospy.get_param('~pub_topic', '/yolov5/BoundingBoxes')
        self.position_pub = rospy.Publisher(
            pub_topic,  BoundingBoxes, queue_size=1)

        #图像发布
        self.image_pub = rospy.Publisher(
            '/yolov5/detection_image',  Image, queue_size=1)

        # 无图像时输出提示信息
        while (not self.getImageStatus) :
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def image_callback(self, image):
        #实例化对象
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        #进入回调函数 将状态置True
        self.getImageStatus = True 
        #字节流转为图像后reshape
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)
        #将CV2输入的BGR格式转为RGB格式
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR)

        results = self.model(self.color_image)
        # xmin    ymin    xmax   ymax  confidence  class    name
        #图像推理结果 将结果转换为Pandas DataFrame 
        boxs = results.pandas().xyxy[0].values

        self.dectshow(self.color_image, boxs, image.height, image.width)

        cv2.waitKey(3)

    def dectshow(self, org_img, boxs, height, width):
        #copy传入的识别的对象 在copy的图像上操作
        img = org_img.copy()
        #记数 作为BoundingBox的num
        count = 0
        #遍历传入的boxs 得到个数
        for i in boxs:
            count += 1
        #遍历识别对象的参数 可能性 左上(x,y) 右下(x,y) 识别对象的数目
        for box in boxs:
            boundingBox = BoundingBox()
            boundingBox.probability =np.float64(box[4])
            boundingBox.xmin = np.int64(box[0])
            boundingBox.ymin = np.int64(box[1])
            boundingBox.xmax = np.int64(box[2])
            boundingBox.ymax = np.int64(box[3])
            boundingBox.num = np.int16(count)
            #识别到的类别
            boundingBox.Class = box[-1]
            #框选的颜色 同一类目标，用同一个颜色的线条画框
            if box[-1] in self.classes_colors.keys():
                color = self.classes_colors[box[-1]]
            else:#没有则随机
                color = np.random.randint(0, 183, 3)
                self.classes_colors[box[-1]] = color
            #对识别到的对象画框 cv2.rectangle(img, pt1, pt2, color[, thickness[, lineType[, shift]]]) 
            #                           左上-> （x,y) (x,y) <-右下   
            cv2.rectangle(img, (int(box[0]), int(box[1])),
                          (int(box[2]), int(box[3])), (int(color[0]),int(color[1]), int(color[2])), 2)
            #防止框选出的文本超出边界
            if box[1] < 20:
                text_pos_y = box[1] + 30 #左边界
            else:
                text_pos_y = box[1] - 10
            #写出图像的类别（文本）
            cv2.putText(img, box[-1],
                        (int(box[0]), int(text_pos_y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

            self.boundingBoxes.class_name = box[-1]
            self.boundingBoxes.bounding_boxes.append(boundingBox)
            #发布对象信息
            self.position_pub.publish(self.boundingBoxes)
        #发布图像信息
        self.publish_image(img, height, width)
        #Imshow出图像
        #cv2.imshow('yoloV5', cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        #cv2.imshow('yolov5',img)
        cv2.waitKey(1)
        # cv2.destroyAllWindows()
    def publish_image(self, imgdata, height, width):
        #实例化
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())#时间戳
        header.frame_id = self.camera_frame#父级
        #高度宽度
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'#图像编码
        #数据转为byte流
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)


def main():
    rospy.init_node('yolov5_ros', anonymous=True)
    yolo_dect = Yolo_Dect()
    rospy.spin()


if __name__ == "__main__":

    main()
