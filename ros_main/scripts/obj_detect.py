#!/usr/bin/env python
import json
import requests
import base64
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

APP_ID = '116095704'
API_KEY = 'bJ2revMUYCDuP3WmQzrMbq9r'
SECRET_KEY = 'NV5xOXAXr8SUZPCPBGOvDk70DMJIQd2e'
token_cache = None
token_expiry = 0
ocr_enabled = False  # 默认启用OCR

def get_access_token():
    global token_cache, token_expiry
    if token_cache is None or rospy.get_time() > token_expiry:
        url = 'https://aip.baidubce.com/oauth/2.0/token'
        data = {
            'grant_type': 'client_credentials',
            'client_id': API_KEY,
            'client_secret': SECRET_KEY
        }
        try:
            req = requests.post(url=url, data=data)
            response = json.loads(req.content)
            token_cache = response['access_token']
            token_expiry = rospy.get_time() + response.get('expires_in', 3600)  # Cache token
        except Exception as e:
            rospy.logerr("Failed to get access token: %s", e)
            return None
    return token_cache

def ocr_image(image):
    token = get_access_token()
    if token is None:
        return []

    ocr_url = f'https://aip.baidubce.com/rest/2.0/image-classify/v2/advanced_general?access_token={token}'
    headers = {'content-type': 'application/x-www-form-urlencoded'}

    # Convert the image to base64
    _, buffer = cv2.imencode('.jpg', image)
    body = base64.b64encode(buffer).decode('utf-8')
    data = {'image': body}
    
    try:
        r = requests.post(url=ocr_url, data=data, headers=headers)
        r.raise_for_status()
        res_words = json.loads(r.content).get('result', [])
        return [item["keyword"] for item in res_words]
    except requests.RequestException as e:
        rospy.logerr("Error in OCR request: %s", e)
        return []

def objdetect_callback(msg):
    global ocr_enabled
    ocr_enable_msg = msg.data  # 更新OCR启用状态
    if ocr_enable_msg == "start":
        ocr_enabled = True
    elif ocr_enable_msg == "stop":
        ocr_enabled = False
    rospy.loginfo("OBJ enabled: %s", ocr_enabled)

def image_callback(img_msg):
    if not ocr_enabled:
        return  # 如果OCR未启用，直接返回
    #print("obj_detect_callback")
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV format
        image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        results = ocr_image(image)
        if results:
            msg = ", ".join(results)
            rospy.loginfo("OBJ Results: %s", msg)
            pub.publish(msg)
    except Exception as e:
        rospy.logerr("Error processing image: %s", e)

def ocr_node():
    global pub
    rospy.init_node('obj_detect', anonymous=True)
    pub = rospy.Publisher('obj_results', String, queue_size=10)
    rospy.Subscriber('/kinect2/hd/image_color_rect', Image, image_callback)
    #rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    rospy.Subscriber('/ObjDetect', String, objdetect_callback)  # 订阅控制开关话题

    rospy.spin()

if __name__ == '__main__':
    try:
        ocr_node()
    except rospy.ROSInterruptException:
        pass
