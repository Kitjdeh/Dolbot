# 이 노드가 가장 먼저 실행되어야 logListId가 생성되고, 생성된 logListId를 이용해서 로그 POST 요청 가능 

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from time import time
from time import localtime
import requests
import json
import base64
import socketio
import os
from . import iot_udp

now_time = time()
now_time=localtime(now_time)

# sio = socketio.Client()
sio=iot_udp.sio


# URL
url = "https://j8a708.p.ssafy.io/api/v1/log/log-lists"

# headers
headers = {
    "Content-Type": "application/json"
}

logListId = 0  # 이 값이 바뀌지가 않음.. json 파일로 저장해야 할듯
class IMGParser(Node):
   

    def __init__(self):
        super().__init__(node_name='image_convertor')
        print("시작")
        self.cnt = 0        
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)

    def img_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # print(msg.data)
        video={
            "id" : 708002,
            "data" : msg.data.tolist()
        }
        # sio.emit('video', json.dumps(video))        

        # file_path=os.path.dirname(os.path.realpath(__file__))
        file_path = os.getcwd() 
        # print(file_path)
        if (self.cnt == 100) :
            cv2.imwrite(file_path+"/img/img_"+str(now_time.tm_year)+str(now_time.tm_mon)+str(now_time.tm_mday)+".PNG", img_bgr)

            picture = ""
            with open(file_path+"/img/img_"+str(now_time.tm_year)+str(now_time.tm_mon)+str(now_time.tm_mday)+".PNG", "rb") as image_file:
                image_base64 = base64.b64encode(image_file.read())
                picture = image_base64.decode('utf-8').replace('+', '-').replace('/', '_')

            mon = str(now_time.tm_mon)
            day = str(now_time.tm_mday)

            if now_time.tm_mon<10:
                mon='0'+mon
            if now_time.tm_mday<10:
                day='0'+day

            logListDto = {
                "logDate": str(now_time.tm_year)+"-"+mon+"-"+day,
                "robotId": 708002,
                "picture" : str(picture)
            }
            print(logListDto)

            data = json.dumps(logListDto)
            response = requests.post(url=url,data=data,headers=headers)

            print("response: ", response)

            temp = response.json()
            print(temp)
            print("logListId: ", temp['logListId'])
            
            global logListId
            logListId = temp['logListId']

            f = open(file_path+"/data/loglistid.txt", 'w')
            f.write(str(logListId))
            f.close()

        self.cnt += 1
    
        cv2.imshow("img_bgr", img_bgr)      
        cv2.waitKey(1)


socket_data = {
    "type": "robot",
    "id": 708002,  # robot ID
    "to": 3,  # user ID
}

def main(args=None):

    # sio.connect('https://j8a708.p.ssafy.io/socket')
    # sio.emit('init_robot', json.dumps(socket_data))
    # sio.emit('targeting', json.dumps(socket_data))

    rclpy.init(args=args)
    image_parser = IMGParser()
    rclpy.spin(image_parser)

if __name__ == '__main__':
    print("메인 시작")
    main()

