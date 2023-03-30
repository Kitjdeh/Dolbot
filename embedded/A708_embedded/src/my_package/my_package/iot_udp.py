
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
from ssafy_msgs.msg import TurtlebotStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist

file_path = os.getcwd() + "/appliance.json"

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
    "tv": "TV가",
    "light": "조명이",
    "air_conditioner": "에어컨이",
    "air_cleaner": "공기청정기가",
    "ON": "켜졌습니다",
    "OFF": "꺼졌습니다"
}


class iot_udp(Node):

    def __init__(self):
        super().__init__('iot_udp')

        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 1)
        self.status_sub = self.create_subscription(
            TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_msg = TurtlebotStatus()
        self.cmd_msg = Twist()
        self.is_status = False

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

        self.is_recv_data = False

        self.is_get_name = False

        os.system('cls')

    def device_control(self, cmd):
        with open(file_path, 'r') as file:
            appliance = json.load(file)
        room_name = cmd["room"]
        device_name = cmd["device"]
        status = cmd["status"]
        mode=""
        speed = ""
        if device_name == "air_conditioner":
            mode = cmd["mode"]
            speed= cmd["speed"]
        elif device_name == "air_cleaner":
            mode = cmd["mode"]

        if appliance[room_name][device_name]["status"] != status:
            goal_pose = PoseStamped()
            goal_pose.pose.position.x = appliance[room_name][device_name]["pose"][0]
            goal_pose.pose.position.y = appliance[room_name][device_name]["pose"][1]
            goal_pose.pose.orientation.w = 1.0
            self.goal_pub.publish(goal_pose)
            if self.is_status == True:
                while not (goal_pose.pose.position.x-0.5 <= self.status_msg.twist.angular.x <= goal_pose.pose.position.x+0.5) or not (goal_pose.pose.position.y-0.5 <= self.status_msg.twist.angular.y <= goal_pose.pose.position.y+0.5):
                    continue
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_msg)
                print("GOAL")
                self.connect()
                self.control()
                self.disconnect()
                appliance[room_name][device_name]["status"] = status
                if device_name == "air_conditioner":
                    appliance[room_name][device_name]["mode"] = mode
                    appliance[room_name][device_name]["speed"] = speed
                elif device_name == "air_cleaner":
                    appliance[room_name][device_name]["mode"] = mode
                global socket_data
                socket_data["message"] = appliance
                sio.emit('home_status', json.dumps(socket_data))
                socket_data["message"] = toast_msg[room_name]+toast_msg[device_name] + \
                    toast_msg[status]
                sio.emit('toast', socket_data)
                print("complete")
            else:
                print("connection error")
            with open(file_path, 'w', encoding='utf-8') as file:
                json.dump(appliance, file, indent="\t")
        else:
            print("already ", status)

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
                    self.send_data(
                        self.recv_data[0], params_control_cmd["RESET"])
            while (params_status[self.recv_data[1]] != "CONNECTION"):
                self.send_data(
                    self.recv_data[0], params_control_cmd["TRY_TO_CONNECT"])
        else:
            print("ERROR : not control area")

    def control(self):
        if self.is_recv_data == True:
            if params_status[self.recv_data[2]] == "OFF":
                while (params_status[self.recv_data[2]] == "OFF"):
                    self.send_data(self.recv_data[0],
                                   params_control_cmd["SWITCH_ON"])
            elif params_status[self.recv_data[2]] == "ON":
                while (params_status[self.recv_data[2]] == "ON"):
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
        time.sleep(0.5)
        self.control()
        time.sleep(0.5)
        self.disconnect()

    def __del__(self):
        self.sock.close()
        print('del')


iot = ''

sio = socketio.Client()

socket_data = {
    "type": "robot",  # !!!t를 type으로 변경했습니다!!!
    "id": 1,  # robot ID
    "to": 1,  # user ID
    "message": "로봇 테스트입니다. 띠디디디-",
}


@sio.event
def connect():
    print('서버에 연결되었습니다.')
    sio.emit('init_robot', json.dumps(socket_data))


@sio.event
def disconnect():
    print('서버와의 연결이 끊어졌습니다.')


@sio.event
def home_status(data):
    print('메시지 수신:', data)
    global socket_data
    with open(file_path, 'r') as file:
        appliance = json.load(file)
    socket_data["message"] = appliance
    sio.emit('home_status', json.dumps(socket_data))


@sio.event
def appliance_status(data):  # !!!chat_message를 robot_message로 변경했습니다!!!
    print('메시지 수신:', data)
    dict = json.loads(data)
    print(dict["type"] + "가 보낸 메세지")
    print("userID="+str(dict["id"]))
    print("robotID="+str(dict["to"]))
    dict2 = dict["message"]
    iot.device_control(dict2)


def main(args=None):
    sio.connect('http://3.36.67.119:8081')
    rclpy.init(args=args)
    global iot
    iot = iot_udp()
    rclpy.spin(iot)
    iot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
