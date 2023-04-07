import rclpy
from rclpy.node import Node
import time
import os
import socket
import threading
import struct
import binascii
import socketio
import json
import numpy as np
import cv2
from time import time
from time import localtime
import requests
import json
import base64
import socketio
import os

from nav_msgs.msg import Path
from ssafy_msgs.msg import TurtlebotStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
from sensor_msgs.msg import CompressedImage
from . import iot_udp


now_time = time()
now_time=localtime(now_time)

# URL
url = "https://j8a708.p.ssafy.io/api/v1/log/log-lists"

# headers
headers = {
    "Content-Type": "application/json"
}

logListId = 0  # 이 값이 바뀌지가 않음.. json 파일로 저장해야 할듯

room_point = {
    "living_room": [-6.209,7.678,-54.168],
    "inner_room": [-9.918,6.298,-144.694],
    "library": [-4.643,13.777,-39.739],
    "small_room": [-6.336,14.727,35.546],
    "kitchen": [-6.480, 7.666, 13.801],
    "entrance": [-6.222,10.832,179.314],
}

act_msg = {
    "living_room": ["거실로", "거실에"],
    "inner_room": ["안방으로", "안방에"],
    "library": ["서재로", "서재에"],
    "small_room": ["작은방으로", "작은방에"],
    "kitchen": ["주방으로", "주방에"],
    "entrance": ["현관으로", "현관에"],
}

user_id=1
socket_data = {
    "type": "robot",  # !!!t를 type으로 변경했습니다!!!
    "id": 708002,  # robot ID
    "to": user_id,  # user ID
    "message": '',
    "room": '',
}

init_data = {  # chan: cctv 프리셋 수정
    "type": "robot",
    "id": 4708002,  # robot ID
    "to": user_id,  # user ID
    "message": "로봇 init",
}

class cctv_cmd(Node):

    def __init__(self):
        super().__init__('cctv_cmd')

        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 1)
        self.status_sub = self.create_subscription(
            TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose2', self.goal_callback, 1)

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed/cctv',
            self.img_callback,
            10)

        self.cnt = 0
        self.status_msg = TurtlebotStatus()
        self.cmd_msg = Twist()
        self.is_status = False
        self.is_goal = False

        self.on_mission = False
    def goal_callback(self, msg):

        if self.is_goal == False:
            self.elder_pose = msg
        self.is_goal = True
        print("human:",msg)
    
    def img_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # print(msg.data)
        video={
            "id": 708002,
            "data" : msg.data.tolist()
        }

        print('emit 비디오 전송 전')  

        socket_data2 = {
            "type": "robot",  # !!!t를 type으로 변경했습니다!!!
            "id": 708002,  # robot ID
            "to": user_id,  # user ID
            "message": '!!!!!!!!!!!!!!video test!!!!!!!!!!!!',
        }

        if self.cnt%50==0:
            # sio.emit('robot_message', json.dumps(socket_data2))
            sio.emit('video', json.dumps(video))     
        print('emit 비디오 전송 후') 
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
    
        cv2.imshow("img_bgr2", img_bgr)      
        cv2.waitKey(1)

    def status_callback(self, msg):
        self.status_msg = msg
        self.is_status = True

    def cctv_control(self, room_name):
        global socket_data
        print("cctv on")
        if room_name == "auto":
            if self.is_goal: #chanyo : cctv auto 모드 구현
                self.goal_pub.publish(self.elder_pose)
            while not (self.elder_pose.pose.position.x-0.1 <= self.status_msg.twist.angular.x <= self.elder_pose.pose.position.x+0.1) or not (self.elder_pose.pose.position.y-0.1 <= self.status_msg.twist.angular.y <= self.elder_pose.pose.position.y+0.1):
                continue
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_msg)
            while not(self.elder_pose.pose.orientation.w-1<=self.status_msg.twist.linear.z<=self.elder_pose.pose.orientation.w+1):
                print("not yet")
                turn_angle = self.elder_pose.pose.orientation.w-self.status_msg.twist.linear.z
                if turn_angle == 0:
                    break
                elif abs(turn_angle)>180 :
                    self.cmd_msg.angular.z = 0.3 * turn_angle/abs(turn_angle)
                else:
                    self.cmd_msg.angular.z = -0.3 *turn_angle/abs(turn_angle)
                self.cmd_msg.linear.x = 0.0
                self.cmd_pub.publish(self.cmd_msg)
            socket_data["message"] = "보호대상자를 관찰중입니다"
            sio.emit('robot_message', json.dumps(socket_data))
            print("auto")
        else:
            print("room")
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_msg)
            goal_pose = PoseStamped()
            goal_pose.pose.position.x = room_point[room_name][0]
            goal_pose.pose.position.y = room_point[room_name][1]
            goal_pose.pose.orientation.w = 1.0
            self.goal_pub.publish(goal_pose)
            self.on_mission = True
            socket_data["message"] = act_msg[room_name][0]+"이동중입니다"
            sio.emit('robot_message', json.dumps(socket_data))
            while not (goal_pose.pose.position.x-0.1 <= self.status_msg.twist.angular.x <= goal_pose.pose.position.x+0.1) or not (goal_pose.pose.position.y-0.1 <= self.status_msg.twist.angular.y <= goal_pose.pose.position.y+0.1):
                continue
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_msg)
            while not(room_point[room_name][2]-1<=self.status_msg.twist.linear.z<=room_point[room_name][2]+1):
                turn_angle = room_point[room_name][2]-self.status_msg.twist.linear.z
                if turn_angle == 0:
                    break
                elif abs(turn_angle)>180 :
                    self.cmd_msg.angular.z = 0.3 * turn_angle/abs(turn_angle)
                else:
                    self.cmd_msg.angular.z = -0.3 *turn_angle/abs(turn_angle)
                self.cmd_msg.linear.x = 0.0
                self.cmd_pub.publish(self.cmd_msg)
            local_path_msg = Path()
            local_path_msg.header.frame_id = '/map'
            self.local_path_pub.publish(local_path_msg)
            print("path reset")
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_msg)
            socket_data["message"] = act_msg[room_name][1]+"도착했습니다"
            sio.emit('robot_message', json.dumps(socket_data))
            print("complete")
            self.on_mission = False

sio = socketio.Client()
# sio = iot_udp.sio
cctv_app = ''

@sio.event
def connect():
    print('서버에 연결되었습니다.')
    sio.emit('init_robot', json.dumps(init_data))
    now_time = time()
    now_time=localtime(now_time)
    print("cctv_init ", now_time)

@sio.event
def disconnect():
    print('서버와의 연결이 끊어졌습니다.')

@sio.event
def robot_message(data):
    global socket_data
    global user_id
    origin = json.loads(data)
    user_id = origin["id"]
    socket_data["to"]=user_id
    print("user_id", str(socket_data["to"]))
    

@sio.event
def cctv(data):
    global socket_data
    global user_id
    origin = json.loads(data)
    print(origin)
    msg = origin["room"]
    user_id = origin["id"]  # chan: cctv소켓에서도 user_id 받음
    socket_data["to"]=user_id
    cctv_app.cctv_control(msg)


def main(args=None):
    sio.connect('https://j8a708.p.ssafy.io/socket')
    rclpy.init(args=args)
    global cctv_app
    cctv_app = cctv_cmd()
    rclpy.spin(cctv_app)
    cctv_app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()