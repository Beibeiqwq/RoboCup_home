import cv2
import face_recognition
import os
import numpy as np
 
# 训练集文件夹的目录
train_dir = r"/home/bei/robot_ws/src/ros_main/face_recognition/train"
 
# 准备空的列表来保存人脸编码和对应的人名
known_face_encodings = []
known_face_names = []
 
# 遍历训练目录中的每一个人
for person in os.listdir(train_dir):
    pix_dir = os.path.join(train_dir, person)
    print(pix_dir)
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
                known_face_encodings.append(face_encoding[0])
                known_face_names.append(person)
 
# # 输出测试
# for i in range(len(known_face_encodings)):
#     print(known_face_encodings[i])
#
# for i in range(len(known_face_names)):
#     print(known_face_names[i])
 
# 写入txt文件
file = open("encoding.txt", "w")
# file.write(known_face_encodings[0])
for i in range(len(known_face_encodings)):
    for j in range(len(known_face_encodings[i])):
        file.write(str(known_face_encodings[i][j]))
        file.write(" ")
    file.write(known_face_names[i])
    file.write("\n")