# -*- coding: utf-8 -*-
import torch
import os
import math
import sys
import cv2
from torch import nn
from torch.autograd import Variable
import matplotlib.pyplot as plt

dir_path = os.path.dirname(os.path.realpath(__file__))

try:
    sys.path.append('/home/bei/openpose/openpose/build/python')
    from openpose import pyopenpose as op

except ImportError as e:
    print('Did you enable `BUILD_PYTHON`')
    raise e


params = dict()
params["model_folder"] = "/home/bei/openpose/openpose/models"
# 根据自己的实际情况选择 model路径
params["number_people_max"] = 1  # 只检测一个人
params["net_resolution"] = "320x176"
params["render_threshold"] = 0.01

opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()

#站立、躺、坐、平坐、行走、蹲起、俯卧撑、摔倒、
#倚靠墙壁、吸烟、打电话、挥手、举手、挥双手
pos = ["站立(正面）", "躺（正面）", "坐（正面）", "平坐", "行走（正面）", "蹲起（正面）",
       "俯卧撑", "摔倒（正面）", "倚靠墙壁", "吸烟", "打电话", "挥手", "举手",
       "挥双手", "站立（侧面）", "躺下（侧面）", "坐（侧面）", "行走（侧面）", "蹲起（侧面）",
       "俯卧撑（侧面）", "摔倒（侧面）", "打电话（侧面）", "吸烟（侧面）", "举手（侧面）"]

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
    
class Video:
    def __init__(self, capture):
        self.capture = capture
        self.currentFrame = None
        self.previousFrame = None

    def captureFrame(self):
        """
        capture frame and return captured frame
        """
        ret, readFrame = self.capture.read()
        return readFrame

    def captureNextFrame(self):
        """
        capture frame and reverse RBG BGR and return opencv image
        """
        ret, readFrame = self.capture.read()
        if ret:
            self.currentFrame = cv2.cvtColor(readFrame, cv2.COLOR_BGR2RGB)

    def convertFrame(self):
        # converts frame to format suitable for QtGui
        try:
            height, width = self.currentFrame.shape[:2]

            self.previousFrame = self.currentFrame
            #return img
        except cv2.Error:
            return None
        


def predict_result(datas=None):
    """
    :param datas: list
    :return: int
    """
    if datas is None:
        datas = []
    model = Classification(30, 200, 300, 100, 24)
    model.load_state_dict(torch.load(
        "../model_pth/ros_pose.pth", map_location='cpu'))
    predict = model(Variable(torch.Tensor([datas]).float())).detach().cpu().numpy().tolist()[0]
    predict = predict.index(max(predict))

    return predict

class MainOpenpose:
    def __init__(self):
          self.video = Video(cv2.VideoCapture(0))

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

    def showCapture(self):
        try:
            #frame = self.video.captureFrame()
            frame = self.video.captureFrame()
            datum = op.Datum()
            datum.cvInputData = frame
            opWrapper.emplaceAndPop(op.VectorDatum([datum]))
            resPic = datum.cvOutputData

            pic = cv2.cvtColor(resPic, cv2.COLOR_BGR2RGB)
            #cv2.imshow("pic",pic)
            if datum.poseKeypoints is None:
                pass
            else:
                #print(f"Pose Keypoints: {datum.poseKeypoints}")
                keyPoints = datum.poseKeypoints.tolist()
                #self.label_4.setText(pos[predict_result(pointDistance(keyPoints[0]) +
                #                                    pointAngle(keyPoints[0]))])
                print(pos[predict_result(self.pointDistance(keyPoints[0]) + self.pointAngle(keyPoints[0]))])

        except TypeError as e:
            print(f"TypeError occurred: {e}")
        except Exception as e:
            print(f"An error occurred: {e}")

if __name__ == '__main__':
    #data = []
    #predict_result(data)
    #print(predict_result(data))
    myOpenpose = MainOpenpose()
    while True:
        myOpenpose.showCapture()


