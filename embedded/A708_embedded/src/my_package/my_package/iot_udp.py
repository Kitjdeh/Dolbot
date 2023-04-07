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
import os
from nav_msgs.msg import Path
from ssafy_msgs.msg import TurtlebotStatus, EnviromentStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist

from . import speech_to_text
import threading

import requests
import json
from time import localtime
from pynput import keyboard

url='https://j8a708.p.ssafy.io'

appliance = {
    "living_room": {  # 거실 (조명/에어컨/TV)
        "light": {"status": "OFF", "pose": [0, 0]},
        "air_conditioner": {"status": "OFF", "mode": "cool", "speed": "mid", "target": 23, "pose": [0, 0]},
        # mode = cool, warm, dry
        # speed = high, mid, low
        "tv": {"status": "OFF", "pose": [-6.36, 15.33]},
    },
    "inner_room": {  # 안방 (조명/에어컨/공기청정기)
        "light": {"status": "OFF", "pose": [0, 0]},
        "air_conditioner": {"status": "OFF", "mode": "cool", "speed": "mid", "pose": [0, 0]},
        "air_cleaner": {"status": "OFF", "mode": "high", "pose": [0, 0]},
        # mode = high, mid, low
    },
    "library": {  # 서재 (조명/공기청정기)
        "light": {"status": "OFF", "pose": [0, 0]},
        "air_cleaner": {"status": "OFF", "pose": [0, 0]},
    },
    "small_room": {  # 작은방 (조명/에어컨/TV)
        "light": {"status": "OFF", "pose": [0, 0]},
        "air_conditioner": {"status": "OFF", "mode": "cool", "speed": "mid", "pose": [0, 0]},
        "tv": {"status": "OFF", "pose": [0, 0]},
    },
    "toilet": {  # 화장실 (조명)
        "light": {"status": "OFF", "pose": [0, 0]},
    },
    "entrance": {  # 현관 (조명)
        "light": {"status": "OFF", "pose": [0, 0]},
    },
}

file_path = os.getcwd()

params_status = {
    (0xa, 0x25): "IDLE",
    (0xb, 0x31): "CONNECTION",
    (0xc, 0x51): "CONNECTION_LOST",
    (0xb, 0x37): "ON",
    (0xa, 0x70): "OFF",
    (0xc, 0x44): "ERROR"
}

params_control_cmd = {
    "TRY_TO_CONNECT": (0xb, 0x31),
    "SWITCH_ON": (0xb, 0x37),
    "SWITCH_OFF": (0xa, 0x70),
    "RESET": (0xb, 0x25),
    "DISCONNECT": (0x00, 0x25)
}

toast_msg = {
    "living_room": "거실",
    "inner_room": "안방",
    "library": "서재",
    "small_room": "작은방",
    "toilet": "화장실",
    "entrance": "현관",
    "tv": ["TV를","TV가"],
    "light": ["조명을","조명이"],
    "air_conditioner": ["에어컨을","에어컨이"],
    "air_cleaner": ["공기청정기를","공기청정기가"],
    "ON": ["켜는중입니다","켜졌습니다"],
    "OFF": ["끄는중입니다","꺼졌습니다"]
}

appliance_mapping={
    "light":1,
    "air_conditioner":2,
    "tv": 3,
    "air_cleaner": 4
}

room_mapping={
    "living_room": 1,
    "inner_room": 2, 
    "library": 3,
    "small_room": 4, 
    "toilet": 5,
    "entrance": 6,
}

# device_pose = {
#     "tv": [-6.36, 16.23],
#     "air": [-12.26, 5.51]
# }

class iot_udp(Node):

    def __init__(self):
        super().__init__('iot_udp')

        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 1)
        self.status_sub = self.create_subscription(
            TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose2', self.goal_callback, 1)

        self.status_msg = TurtlebotStatus()
        self.cmd_msg = Twist()
        self.elder_pose = PoseStamped()
        self.is_status = False
        self.is_drive = False
        self.is_goal = False

        self.ip = '127.0.0.1'
        self.port = 7502
        self.send_port = 7401

        # 로직 1. 통신 소켓 생성
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (self.ip, self.port)
        self.sock.bind(recv_address)
        self.data_size = 65535
        self.parsed_data = []

        # 로직 2. 멀티스레드를 이용한 데이터 수신
        thread = threading.Thread(target=self.recv_udp_data)
        thread.daemon = True
        thread.start()

        # key 입력 스레드 생성
        key_t = threading.Thread(target=self.keyevent)
        key_t.daemon = True
        key_t.start()

        self.is_recv_data = False
        self.is_get_name = False

        # os.system('cls')

    def device_control(self, cmd):
        with open(file_path+ "/appliance.json", 'r') as file:
            appliance = json.load(file)
        room_name = cmd["room"]
        device_name = cmd["device"]
        status = cmd["status"]
        mode=""
        speed = ""
        target=""
        if device_name == "air_conditioner" and status == "ON":
            mode = cmd["mode"]
            speed= cmd["speed"]
            target = cmd["target"]
        elif device_name == "air_cleaner" and status=="ON":
            mode = cmd["mode"]

        if appliance[room_name][device_name]["status"] != status:
            print("path planning....")
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_msg)
            time.sleep(0.5)
            start_pose = PoseStamped()
            start_pose.pose.position.x = self.status_msg.twist.angular.x
            start_pose.pose.position.y = self.status_msg.twist.angular.y
            start_pose.pose.orientation.w = 1.0
            goal_pose = PoseStamped()
            goal_pose.pose.position.x = appliance[room_name][device_name]["pose"][0]
            goal_pose.pose.position.y = appliance[room_name][device_name]["pose"][1]
            goal_pose.pose.orientation.w = 1.0
            self.goal_pub.publish(goal_pose)
            self.is_drive = True
            global socket_data
            socket_data["message"] = toast_msg[room_name]+toast_msg[device_name][0] + \
                    toast_msg[status][0]
            sio.emit('robot_message',json.dumps(socket_data))

            if self.is_status == True:
                while not (goal_pose.pose.position.x-0.1 <= self.status_msg.twist.angular.x <= goal_pose.pose.position.x+0.1) or not (goal_pose.pose.position.y-0.1 <= self.status_msg.twist.angular.y <= goal_pose.pose.position.y+0.1):
                    continue

                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_msg)
                time.sleep(0.5)
                print("GOAL")
                self.connect()
                self.control()
                self.disconnect()

                appliance[room_name][device_name]["status"] = status
                if device_name == "air_conditioner" and status == "ON":
                    appliance[room_name][device_name]["mode"] = mode
                    appliance[room_name][device_name]["speed"] = speed
                    appliance[room_name][device_name]["target"] = target
                elif device_name == "air_cleaner" and status == "ON":
                    appliance[room_name][device_name]["mode"] = mode

                socket_data["message"] = appliance
                sio.emit('home_status', json.dumps(socket_data))
                socket_data["message"] = toast_msg[room_name]+toast_msg[device_name][1] + \
                    toast_msg[status][1]
                sio.emit('toast', json.dumps(socket_data))
                sio.emit('robot_message',json.dumps(socket_data))
                self.is_drive = False
                print("complete")

                # 가전제어 완료 로그 POST 요청 
                f = open(file_path+"/data/loglistid.txt", 'r')
                loglistid = f.read() 
                f.close()

                now_time = time.time()
                now_time=localtime(now_time)
                body = {
                    "applianceId": appliance_mapping[device_name],  # 가전기기마다 번호 매핑
                    "logListId": loglistid,  # 최초 사진 찍을 때 loglistid 생성 => 받아와서 값 넣어주기 
                    "logTime": str(now_time.tm_hour)+":"+str(now_time.tm_min)+":"+str(now_time.tm_sec),
                    "on": True if status=='ON' else False,
                    "roomId": room_mapping[room_name]  # 방마다 번호 매핑
                }
                res = requests.post(
                    url+'/api/v1/log/log/appliance-log',
                    headers={'Content-Type': 'application/json', 'charset': 'UTF-8', 'Accept': '*/*'},
                    data=json.dumps(body)
                )
                print(loglistid)
                print(res)

                return_p = PoseStamped()
                return_p.pose.position.x =100.0
                return_p.pose.position.y =100.0
                return_p.pose.orientation.w =1.0
                self.goal_pub.publish(return_p)
                self.is_drive = True
            
                while not (start_pose.pose.position.x-0.1 <= self.status_msg.twist.angular.x <= start_pose.pose.position.x+0.1) or not (start_pose.pose.position.y-0.1 <= self.status_msg.twist.angular.y <= start_pose.pose.position.y+0.1):
                    if self.is_goal: #chan: 복귀중 객체 추종
                        print("go to human")
                        self.goal_pub.publish(self.elder_pose)
                        print(self.elder_pose)
                        while not (self.elder_pose.pose.position.x-0.1 <= self.status_msg.twist.angular.x <= self.elder_pose.pose.position.x+0.1) or not (self.elder_pose.pose.position.y-0.1 <= self.status_msg.twist.angular.y <= self.elder_pose.pose.position.y+0.1):
                            continue
                        self.is_goal = False
                        break
                    continue
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_msg)
                time.sleep(0.5) #chan: follow 안되게 하던 버그 수정
                print("return finish")
                if self.is_goal:
                    print("go to human")
                    self.goal_pub.publish(self.elder_pose)
                    print(self.elder_pose)
                    while not (self.elder_pose.pose.position.x-0.1 <= self.status_msg.twist.angular.x <= self.elder_pose.pose.position.x+0.1) or not (self.elder_pose.pose.position.y-0.1 <= self.status_msg.twist.angular.y <= self.elder_pose.pose.position.y+0.1):
                        continue
                    self.is_goal = False
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 0.0
                    self.cmd_pub.publish(self.cmd_msg)
                print("return")

            else:
                print("connection error")

            with open(file_path+ "/appliance.json", 'w', encoding='utf-8') as file:
                json.dump(appliance, file, indent="\t")
        else:
            if device_name == "air_conditioner" and status == "ON":
                appliance[room_name][device_name]["mode"] = mode
                appliance[room_name][device_name]["speed"] = speed
                appliance[room_name][device_name]["target"] = target
            elif device_name == "air_cleaner" and status == "ON":
                appliance[room_name][device_name]["mode"] = mode
            with open(file_path+ "/appliance.json", 'w', encoding='utf-8') as file:
                json.dump(appliance, file, indent="\t")
        print("already ", status)

    def goal_callback(self, msg):

        if self.is_goal == False:
            self.elder_pose = msg
        self.is_goal = True
        print("human:",msg)


    def status_callback(self, msg):
        self.status_msg = msg
        self.is_status = True

    def data_parsing(self, raw_data):
        # print(raw_data)
        # 예시 raw_data = b'#Appliances-Status$\x14\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xe0\xec\xab\xd0;\xc3H8\x94\xd0\xed\x07\xde\x12n\xab\n%\np\r\n'
        # 로직 3. 수신 데이터 파싱

        header = raw_data[0:19].decode()
        data_length = struct.unpack('i', raw_data[19:23])
        # aux_data=struct.unpack('i',raw_data[19:23])

        if header == '#Appliances-Status$' and data_length[0] == 20:
            uid_pack = struct.unpack('16B', raw_data[23+12:39+12])
            uid = self.packet_to_uid(uid_pack)
            # 예시 uid = e0ecabd03bc3483894d0ed07de126eab0a250a70

            network_status = struct.unpack('2B', raw_data[39+12:41+12])
            device_status = struct.unpack('2B', raw_data[41+12:43+12])

            self.is_recv_data = True
            self.recv_data = [uid, network_status, device_status]

    def send_data(self, uid, cmd):

        # pass
        # 로직 4. 데이터 송신 함수 생성

        Dheader = '#Ctrl-command$'
        header = Dheader.encode()
        data_length = struct.pack('i', 18)
        aux_data = struct.pack('iii', 0, 0, 0)
        self.upper = header + data_length+aux_data
        self.tail = '\r\n'.encode()
        print('tail:', self.tail)
        print('length:', data_length)
        print('header:', header)

        uid_pack = self.uid_to_packet(uid)
        print('uid:', uid_pack)
        cmd_pack = bytes([cmd[0], cmd[1]])
        print('cmd:', cmd_pack)

        send_data = self.upper+uid_pack+cmd_pack+self.tail
        # print(send_data)
        self.sock.sendto(send_data, (self.ip, self.send_port))

    def recv_udp_data(self):
        while True:
            raw_data, sender = self.sock.recvfrom(self.data_size)
            self.data_parsing(raw_data)

    def uid_to_packet(self, uid):
        uid_pack = binascii.unhexlify(uid)
        return uid_pack

    def packet_to_uid(self, packet):
        uid = ""
        for data in packet:
            if len(hex(data)[2:4]) == 1:
                uid += "0"

            uid += hex(data)[2:4]

        return uid

    def scan(self):

        print('SCANNING NOW.....')
        print('BACK TO MENU : Ctrl+ C')
        '''
        로직 6. iot scan

        주변에 들어오는 iot 데이터(uid,network status, device status)를 출력하세요.

        '''
        if self.is_recv_data == True:
            print("UID:", self.recv_data[0])
            print("networt status:", self.recv_data[1])
            print("device status:", self.recv_data[2])

    def connect(self):

        if self.is_recv_data == True:
            if params_status[self.recv_data[1]] == "CONNECTION_LOST":
                while (params_status[self.recv_data[1]] == "CONNECTION_LOST"):
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 1.0
                    self.cmd_pub.publish(self.cmd_msg)
                    self.send_data(
                        self.recv_data[0], params_control_cmd["RESET"])
            while (params_status[self.recv_data[1]] != "CONNECTION"):
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 1.0
                self.cmd_pub.publish(self.cmd_msg)
                self.send_data(
                    self.recv_data[0], params_control_cmd["TRY_TO_CONNECT"])
        else:
            print("ERROR : not control area")

    def control(self):
        if self.is_recv_data == True:
            if params_status[self.recv_data[2]] == "OFF":
                while (params_status[self.recv_data[2]] == "OFF"):
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 1.0
                    self.cmd_pub.publish(self.cmd_msg)
                    self.send_data(self.recv_data[0],
                                   params_control_cmd["SWITCH_ON"])
            elif params_status[self.recv_data[2]] == "ON":
                while (params_status[self.recv_data[2]] == "ON"):
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 1.0
                    self.cmd_pub.publish(self.cmd_msg)
                    self.send_data(self.recv_data[0],
                                   params_control_cmd["SWITCH_OFF"])
            else:
                print("ERROR : can not get device status")
        else:
            print("ERROR : not control area")

    def disconnect(self):
        if self.is_recv_data == True:
            self.send_data(self.recv_data[0], params_control_cmd["DISCONNECT"])

    def all_procedures(self):
        self.connect()
        time.time.sleep(0.5)
        self.control()
        time.time.sleep(0.5)
        self.disconnect()

    def __del__(self):
        self.sock.close()
        print('del')

    # Key 입력 스레드 관련 메서드
    def keyevent(self):
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()
    
    def on_press(self, key):
        if "Key.space" == str(key):
            print('space bar click!!!')

            # txt 파일에 True -> False, False -> True
            # speech_to_text에서 파일의 값이 True일때만 음성 출력
            key_status=''
            f1 = open(file_path+"/data/key_status.txt", 'r')
            key_status = f1.read() 
            f1.close()
           
            key_now_status=''
            if key_status=='False': # chan: 스페이스바를 누를때 켜짐/ 상태 즉각 저장
                key_now_status='True'
                f2 = open(file_path+"/data/key_status.txt", 'w')
                f2.write(key_now_status)
                f2.close()
                t = threading.Thread(target=speech_to_text.speechToText)
                t.start()
            elif key_status=='True':
                key_now_status='False'
                f2 = open(file_path+"/data/key_status.txt", 'w')
                f2.write(key_now_status)
                f2.close()
            print("이전 : ", key_status)
            print('현재 : ', key_now_status)

            # f2 = open(file_path+"/data/key_status.txt", 'w')
            # f2.write(key_now_status)
            # f2.close()
            


iot = ''

sio = socketio.Client()
user_id=0

socket_data = {
    "type": "robot",
    "id": 708002,  # robot ID
    "to": user_id,  # user ID
    "message": "",
}

init_data = {
    "type": "robot",
    "id": 1708002,  # robot ID
    "to": user_id,  # user ID
    "message": "로봇 init",
}


@sio.event
def connect():
    print('서버에 연결되었습니다.')
    sio.emit('init_robot', json.dumps(init_data))
    now_time = time.time()
    now_time=localtime(now_time)
    print("cctv_init ", now_time)



@sio.event
def disconnect():
    print('서버와의 연결이 끊어졌습니다.')


@sio.event
def robot_message(data):
    origin = json.loads(data)
    global user_id
    user_id=origin['id']

    print('메시지 수신:', data)
    global socket_data
    socket_data['to']=user_id

    if origin["message"]=="True":
        with open(file_path+ "/appliance.json", 'r') as file:
            appliance = json.load(file)
        socket_data["message"] = appliance
        sio.emit('home_status', json.dumps(socket_data))

    print("user_id", str(socket_data["to"]))

@sio.event
def appliance_status(data): 
    dict = json.loads(data)
    global user_id
    user_id=dict['id']

    print('메시지 수신:', dict)
    global socket_data
    socket_data['to']=user_id

    print(dict["type"] + "가 보낸 메세지")
    print("userID="+str(dict["id"]))
    print("robotID="+str(dict["to"]))
    
    dict2 = dict["message"]
    iot.device_control(dict2)


def main(args=None):

    # STT Thread (관련 파일은 import 해서 사용)
    t = threading.Thread(target=speech_to_text.speechToText)
    t.start()

    sio.connect('https://j8a708.p.ssafy.io/socket')
    rclpy.init(args=args)
    global iot
    iot = iot_udp()

    rclpy.spin(iot)
    iot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
