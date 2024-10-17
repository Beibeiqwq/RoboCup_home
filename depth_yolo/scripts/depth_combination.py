#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import pcl
from yolov5_ros.msg import BoundingBox, BoundingBoxes
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
import math
import tf
import tf2_ros

height = 540
width = 960
if_pcl_ready = 0
parent_frame = "kinect2_ir_optical_frame"

def yolov5_callback(data):
    global p,if_pcl_ready
    
    obj_tf = tf.TransformBroadcaster()
    if(if_pcl_ready):
        bounding_boxes = data.bounding_boxes
        id = 0
        for i in bounding_boxes:
            x = 0
            y = 0
            z = 0
            valid_num = 0
            length = i.xmax - i.xmin
            height = i.ymax - i.ymin
            for ix in range(int(i.xmin+length/4),int(i.xmax-length/2)):
                for iy in range(int(i.ymin+height/4),int(i.ymax-height/4)):
                    index = int((iy-1)*width+ix)
                    position = p[index]
                    if not(math.isnan(position[2])):
                        x = x + position[0]
                        y = y + position[1]
                        z = z + position[2]
                        valid_num = valid_num + 1
            if(valid_num):
                x = x/valid_num
                y = y/valid_num
                z = z/valid_num
            #obj_tf.sendTransform((z, -x, -y),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),i.Class+str(id),parent_frame)
            obj_tf.sendTransform((x, y, z),tf.transformations.quaternion_from_euler(0, -math.pi/2, math.pi/2),rospy.Time.now(),i.Class+str(id),parent_frame)
            id = id + 1
        

def depth_callback(data):
    global p,if_pcl_ready
    pc = ros_numpy.numpify(data)
    np_points = np.zeros((height * width, 3), dtype=np.float32)
    np_points[:, 0] = np.resize(pc['x'], height * width)
    np_points[:, 1] = np.resize(pc['y'], height * width)
    np_points[:, 2] = np.resize(pc['z'], height * width)
    p = pcl.PointCloud(np.array(np_points, dtype=np.float32))
    if_pcl_ready = 1
    
def listener():
    rospy.init_node('depth_combination', anonymous=True)
    rospy.Subscriber("/yolov5/BoundingBoxes", BoundingBoxes, yolov5_callback)
    rospy.Subscriber("/kinect2/qhd/points", PointCloud2, depth_callback)
    rospy.spin()
 
def main():
    p = pcl.PointCloud()
    listener()

if __name__ == '__main__':
    main()

