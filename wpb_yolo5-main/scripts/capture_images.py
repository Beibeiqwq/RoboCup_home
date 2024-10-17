#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

capture_one_frame = False
image_index = 1

# 指令回调函数
def cbCommand(msg):
    # rospy.logwarn("指令 = %s", msg.data)
    if msg.data == "capture image":
        global capture_one_frame
        capture_one_frame = True

# 彩色图像回调函数
def cbImage(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    # 弹出窗口显示图片
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(1)
    # 保存图像到文件
    global capture_one_frame
    if capture_one_frame == True:
        while True:
            global image_index
            image_filename = images_path + '/' + "%.04d" % image_index + '.jpg'
            image_index += 1
            if os.path.exists(image_filename):
                continue
            cv2.imwrite(image_filename, cv_image)
            rospy.logwarn("保存图片:" + image_filename)
            capture_one_frame = False
            break

# 主函数
if __name__ == "__main__":
    rospy.init_node("capture_images")
    # 检测并创建 capture 文件夹
    home_path = os.environ['HOME']
    capture_path = home_path + '/capture_images'
    old_umask = os.umask(0o022) # u=rwx,g=rx,o=rx
    try:
        if not os.path.exists(capture_path):
            rospy.logwarn("创建文件夹：" + capture_path)
            os.makedirs(capture_path)
    except IOError as e:
        print('ERROR:创建 %s 失败:\n\t%s' % (path, e), file=sys.stderr)
    except OSError as e:
        print("ERROR: 创建 %s 失败:\n\t%s\n" % (path, e), file=sys.stderr)
    finally:
        os.umask(old_umask)
    
    # 检测并创建 images 图片文件夹
    global images_path
    images_path = capture_path + '/images'
    old_umask = os.umask(0o022) # u=rwx,g=rx,o=rx
    try:
        if not os.path.exists(images_path):
            rospy.logwarn("创建图片文件夹：" + images_path)
            os.makedirs(images_path)
    except IOError as e:
        print('ERROR:创建 %s 失败:\n\t%s' % (path, e), file=sys.stderr)
    except OSError as e:
        print("ERROR: 创建 %s 失败:\n\t%s\n" % (path, e), file=sys.stderr)
    finally:
        os.umask(old_umask)
    
    # 检测并创建 Annotations 标注文件夹
    anno_path = capture_path + '/Annotations'
    old_umask = os.umask(0o022) # u=rwx,g=rx,o=rx
    try:
        if not os.path.exists(anno_path):
            rospy.logwarn("创建标注文件夹：" + anno_path)
            os.makedirs(anno_path)
    except IOError as e:
        print('ERROR:创建 %s 失败:\n\t%s' % (path, e), file=sys.stderr)
    except OSError as e:
        print("ERROR: 创建 %s 失败:\n\t%s\n" % (path, e), file=sys.stderr)
    finally:
        os.umask(old_umask)    

    # 订阅机器人视觉传感器的图像话题
    # image_sub = rospy.Subscriber("/kinect2/hd/image_color_rect",Image,cbImage,queue_size=10)
    image_sub = rospy.Subscriber("/camera/color/image_raw",Image,cbImage,queue_size=10)
    command_sub = rospy.Subscriber("/wpb_yolo5/command",String,cbCommand,queue_size=1)
    rospy.spin()