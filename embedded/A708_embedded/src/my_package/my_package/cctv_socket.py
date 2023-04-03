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
from nav_msgs.msg import Path
from ssafy_msgs.msg import TurtlebotStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist

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

socket_data = {
    "type": "robot",  # !!!t를 type으로 변경했습니다!!!
    "id": 1,  # robot ID
    "to": 1,  # user ID
    "message": "로봇 테스트입니다. 띠디디디-",
}


class cctv_cmd(Node):

    def __init__(self):
        super().__init__('cctv_cmd')

        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 1)
        self.status_sub = self.create_subscription(
            TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)

        self.status_msg = TurtlebotStatus()
        self.cmd_msg = Twist()
        self.is_status = False

        self.on_mission = False

    def status_callback(self, msg):
        self.status_msg = msg
        self.is_status = True

    def cctv_control(self, room_name):
        global socket_data
        if room_name == "auto":
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
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.3
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
cctv = ''

@sio.event
def connect():
    print('서버에 연결되었습니다.')
    sio.emit('init_robot', json.dumps(socket_data))


@sio.event
def disconnect():
    print('서버와의 연결이 끊어졌습니다.')


@sio.event
def appliance_status(data):
    origin = json.loads(data)
    msg = origin["message"]
    cctv.cctv_control(msg)


def main(args=None):
    sio.connect('https://j8a708.p.ssafy.io/socket')
    rclpy.init(args=args)
    global cctv
    cctv = cctv_cmd()
    rclpy.spin(cctv)
    cctv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()