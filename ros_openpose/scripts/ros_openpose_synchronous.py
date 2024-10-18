#!/usr/bin/env python

# import modules
import sys
import cv2
import rospy
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

#站立、躺、坐、平坐、行走、蹲起、俯卧撑、摔倒、
#倚靠墙壁、吸烟、打电话、挥手、举手、挥双手
pos = ["站立(正面）", "躺（正面）", "坐（正面）", "平坐", "行走（正面）", "蹲起（正面）",
       "俯卧撑", "摔倒（正面）", "倚靠墙壁", "吸烟", "打电话", "挥手", "举手",
       "挥双手", "站立（侧面）", "躺下（侧面）", "坐（侧面）", "行走（侧面）", "蹲起（侧面）",
       "俯卧撑（侧面）", "摔倒（侧面）", "打电话（侧面）", "吸烟（侧面）", "举手（侧面）"]

# Import Openpose (Ubuntu)
rospy.init_node('ros_openpose')
py_openpose_path = rospy.get_param("~py_openpose_path")

#dir_path = os.path.dirname(os.path.realpath(__file__))

try:
    # If you run `make install` (default path is `/usr/local/python` for Ubuntu)
    sys.path.append(py_openpose_path)
    from openpose import pyopenpose as op
except ImportError as e:
    rospy.logerr('OpenPose library could not be found. '
                 'Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
    raise e


OPENPOSE1POINT7_OR_HIGHER = 'VectorDatum' in op.__dict__
class Classification(nn.Module):
    def __init__(self, in_dim, n_hidden_1, n_hidden_2, n_hidden_3, out_dim):
        super(Classification, self).__init__()
        self.layer1 = nn.Sequential(
            nn.Linear(in_dim, n_hidden_1), nn.ReLU(True))
        self.layer2 = nn.Sequential(
            nn.Linear(n_hidden_1, n_hidden_2), nn.ReLU(True))
        self.layer3 = nn.Sequential(
            nn.Linear(n_hidden_2, n_hidden_3), nn.ReLU(True))
        self.layer4 = nn.Sequential(nn.Linear(n_hidden_3, out_dim))

    def forward(self, x):
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)

        return x
    

def predict_result(datas=None):
    """
    :param datas: list
    :return: int
    """
    if datas is None:
        datas = []
    model = Classification(30, 200, 300, 100, 24)
    model.load_state_dict(torch.load(
        "/home/bei/robot_ws/src/ros_openpose/model_pth/ros_pose.pth", map_location='cpu'))
    predict = model(Variable(torch.Tensor([datas]).float())).detach().cpu().numpy().tolist()[0]
    predict = predict.index(max(predict))

    return predict


class rosOpenPose:
    def __init__(self, frame_id, no_depth, pub_topic, color_topic, depth_topic, cam_info_topic, op_wrapper, display):

        self.pub = rospy.Publisher(pub_topic, Frame, queue_size=10)

        self.frame_id = frame_id
        self.no_depth = no_depth

        self.bridge = CvBridge()

        self.op_wrapper = op_wrapper

        self.display = display
        self.frame = None

        # Populate necessary K matrix values for 3D pose computation.
        cam_info = rospy.wait_for_message(cam_info_topic, CameraInfo)
        self.fx = cam_info.K[0]
        self.fy = cam_info.K[4]
        self.cx = cam_info.K[2]
        self.cy = cam_info.K[5]

        # Obtain depth topic encoding
        encoding = rospy.wait_for_message(depth_topic, Image).encoding
        self.mm_to_m = 0.001 if encoding == "16UC1" else 1.

        # Function wrappers for OpenPose version discrepancies
        if OPENPOSE1POINT7_OR_HIGHER:
            self.emplaceAndPop = lambda datum: self.op_wrapper.emplaceAndPop(op.VectorDatum([datum]))
            self.detect = lambda kp: kp is not None
        else:
            self.emplaceAndPop = lambda datum: self.op_wrapper.emplaceAndPop([datum])
            self.detect = lambda kp: kp.shape != ()

        image_sub = message_filters.Subscriber(color_topic, Image)
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 1, 0.01)
        self.ts.registerCallback(self.callback)

    def compute_3D_vectorized(self, kp, depth):
        # Create views (no copies made, so this remains efficient)
        U = kp[:, :, 0]
        V = kp[:, :, 1]

        # Extract the appropriate depth readings
        num_persons, body_part_count = U.shape
        XYZ = np.zeros((num_persons, body_part_count, 3), dtype=np.float32)
        for i in range(num_persons):
            for j in range(body_part_count):
                u, v = int(U[i, j]), int(V[i, j])
                if v < depth.shape[0] and u < depth.shape[1]:
                    XYZ[i, j, 2] = depth[v, u]

        XYZ[:, :, 2] *= self.mm_to_m  # convert to meters

        # Compute 3D coordinates in vectorized way
        Z = XYZ[:, :, 2]
        XYZ[:, :, 0] = (Z / self.fx) * (U - self.cx)
        XYZ[:, :, 1] = (Z / self.fy) * (V - self.cy)
        return XYZ
    
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


    def callback(self, ros_image, ros_depth):
        # Construct a frame with current time !before! pushing to OpenPose
        fr = Frame()
        fr.header.frame_id = self.frame_id
        fr.header.stamp = rospy.Time.now()

        # Convert images to cv2 matrices
        image = depth = None
        try:
            image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            depth = self.bridge.imgmsg_to_cv2(ros_depth, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # Push data to OpenPose and block while waiting for results
        try:
            datum = op.Datum()
            datum.cvInputData = image
            self.emplaceAndPop(datum)
        except Exception as e:
            print(f"An error occurred: {e}")
        pose_kp = datum.poseKeypoints
        lhand_kp = datum.handKeypoints[0]
        rhand_kp = datum.handKeypoints[1]
        # keyPoints = datum.poseKeypoints.tolist()
        # print(pos[predict_result(self.pointDistance(keyPoints[0]) + self.pointAngle(keyPoints[0]))])
        if datum.poseKeypoints is None:
                pass
        else:
                #print(f"Pose Keypoints: {datum.poseKeypoints}")
                keyPoints = datum.poseKeypoints.tolist()
                #self.label_4.setText(pos[predict_result(pointDistance(keyPoints[0]) +
                #                                    pointAngle(keyPoints[0]))])
                try:
                    print(pos[predict_result(self.pointDistance(keyPoints[0]) + self.pointAngle(keyPoints[0]))])
                
                except Exception as e:
                    print(f"An error occurred: {e}")

        
        # Set number of people detected
        if self.detect(pose_kp):
            num_persons = pose_kp.shape[0]
            body_part_count = pose_kp.shape[1]
        else:
            num_persons = 0
            body_part_count = 0

        # Check to see if hands were detected
        lhand_detected = False
        rhand_detected = False
        hand_part_count = 0

        if self.detect(lhand_kp):
            lhand_detected = True
            hand_part_count = lhand_kp.shape[1]

        if self.detect(rhand_kp):
            rhand_detected = True
            hand_part_count = rhand_kp.shape[1]

        # Handle body points
        
        fr.persons = [Person() for _ in range(num_persons)]
        if num_persons != 0:
            # Perform vectorized 3D computation for body keypoints
            b_XYZ = self.compute_3D_vectorized(pose_kp, depth)

            # Perform the vectorized operation for left hand
            if lhand_detected:
                lh_XYZ = self.compute_3D_vectorized(lhand_kp, depth)

            # Do same for right hand
            if rhand_detected:
                rh_XYZ = self.compute_3D_vectorized(rhand_kp, depth)

            for person in range(num_persons):
                fr.persons[person].bodyParts = [BodyPart() for _ in range(body_part_count)]
                fr.persons[person].leftHandParts = [BodyPart() for _ in range(hand_part_count)]
                fr.persons[person].rightHandParts = [BodyPart() for _ in range(hand_part_count)]

                detected_hands = []
                if lhand_detected:
                    detected_hands.append((lhand_kp, fr.persons[person].leftHandParts, lh_XYZ))
                if rhand_detected:
                    detected_hands.append((rhand_kp, fr.persons[person].rightHandParts, rh_XYZ))

                # Process the body
                for bp in range(body_part_count):
                    u, v, s = pose_kp[person, bp]
                    x, y, z = b_XYZ[person, bp]
                    arr = fr.persons[person].bodyParts[bp]
                    arr.pixel.x = u
                    arr.pixel.y = v
                    arr.score = s
                    arr.point.x = x
                    arr.point.y = y
                    arr.point.z = z

                # Process left and right hands
                for kp, harr, h_XYZ in detected_hands:
                    for hp in range(hand_part_count):
                        u, v, s = kp[person, hp]
                        x, y, z = h_XYZ[person, hp]
                        arr = harr[hp]
                        arr.pixel.x = u
                        arr.pixel.y = v
                        arr.score = s
                        arr.point.x = x
                        arr.point.y = y
                        arr.point.z = z
        try:
            if self.display: self.frame = datum.cvOutputData.copy()
        except Exception as e:
            print(f"An error occurred: {e}")
        self.pub.publish(fr)

def main():
    frame_id = rospy.get_param("~frame_id")
    no_depth = rospy.get_param("~no_depth")
    pub_topic = rospy.get_param("~pub_topic")
    color_topic = rospy.get_param("~color_topic")
    depth_topic = rospy.get_param("~depth_topic")
    cam_info_topic = rospy.get_param("~cam_info_topic")
    try:
        # Flags, refer to include/openpose/flags.hpp for more parameters
        parser = argparse.ArgumentParser()
        args = parser.parse_known_args()

        # Custom Params
        params = dict()
        # Can manually set params like this as well
        # params["model_folder"] = "/home/asjchoi/Programs/openpose-1.6.0/models"

        # Any more obscure flags can be found through this for loop
        for i in range(0, len(args[1])):
            curr_item = args[1][i]
            if i != len(args[1])-1: next_item = args[1][i+1]
            else: next_item = "1"
            if "--" in curr_item and "--" in next_item:
                key = curr_item.replace('-', '')
                if key not in params:  params[key] = "1"
            elif "--" in curr_item and "--" not in next_item:
                key = curr_item.replace('-', '')
                if key not in params: params[key] = next_item

        # Starting OpenPose
        op_wrapper = op.WrapperPython()
        op_wrapper.configure(params)
        op_wrapper.start()

        display = True if 'display' not in params or int(params['display']) > 0 else False

        # Start ros wrapper
        rop = rosOpenPose(frame_id, no_depth, pub_topic, color_topic, depth_topic, cam_info_topic, op_wrapper, display)

        if display:
            while not rospy.is_shutdown():
                if rop.frame is not None:
                    #cv2.imshow("Ros OpenPose", rop.frame)
                    cv2.waitKey(1)
        else:
            rospy.spin()

    except Exception as e:
        print(f"An error occurred: {e}")
        rospy.logerr(e)
        sys.exit(-1)


if __name__ == "__main__":
    main()
