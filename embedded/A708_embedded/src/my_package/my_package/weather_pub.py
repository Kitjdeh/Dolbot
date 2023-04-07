
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
from ssafy_msgs.msg import EnviromentStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
import os
# from . import iot_udp

file_path = os.getcwd()+"/appliance.json"

air_conditioner = {  # 에어컨
    "mode": {
        "cool": -1,
        "warm": 1,
        "dry": 0
    },
    "speed": {
        "high": 1,
        "mid": 0.5,
        "low": 0.1
    },
}

air_cleaner = {  # 공기청정기
    "mode": {
        "high": 1,
        "mid": 0.5,
        "low": 0.1
    }
}

humidity = {  # 날씨에 따른 기본 습도
    "Sunny": 30,
    "Cloudy": 60,
    "Foggy": 60,
    "Stormy": 70,
    "Rainy": 80,
    "Snowy": 70
}

air_condition = {  # 날씨에 따른 기본 공기 질
    "Sunny": 30,
    "Cloudy": 60,
    "Foggy": 60,
    "Stormy": 70,
    "Rainy": 80,
    "Snowy": 70
}

# 환경변수
out_temp = 0
out_hum = 0
in_temp = None
in_hum = None
air =0
environment = ''

before_environment = 'bad'
# sio=iot_udp.sio

class weather_pub(Node):

    def __init__(self):
        super().__init__('weather_pub')

        self.envir_sub = self.create_subscription(
            EnviromentStatus, '/envir_status', self.envir_status_callback, 10)
        self.envir_msg = EnviromentStatus()
        self.get_weather = False

        thread = threading.Thread(target=self.set_envir)
        thread.daemon = True
        thread.start()

    def envir_status_callback(self, msg):  # 시뮬레이터에서 날씨를 받아 환경변수 셋팅
        self.envir_msg = msg
        global out_temp, out_hum, environment, in_temp, in_hum, air, before_environment
        out_temp = msg.temperature
        if in_temp == None:
            in_temp = out_temp
        environment = msg.weather
        air = air_condition[environment]
        out_hum = humidity[environment]
        if in_hum == None:
            in_hum = out_hum
        self.get_weather = True
        if environment != before_environment:
            print("setting complete")
            before_environment = environment
            self.publish_weather()

    def set_envir(self):  # 에어컨/공기청정기 작동여부에 따른 실내환경 설정
        while True:
            global in_temp, in_hum, air
            if self.get_weather == True:
                with open(file_path, 'r') as file:
                    appliance = json.load(file)
                aircon_status = appliance["living_room"]["air_conditioner"]
                if aircon_status["status"] == "ON":
                    print("air_ON")
                    in_temp += air_conditioner["mode"][aircon_status["mode"]] * \
                        air_conditioner["speed"][aircon_status["speed"]]
                    if aircon_status["mode"] == "cool" and in_temp < aircon_status["target"]:
                        in_temp = aircon_status["target"]
                    elif aircon_status["mode"] == "warm" and in_temp > aircon_status["target"]:
                        in_temp = aircon_status["target"]

                    # 제습모드
                    if aircon_status["mode"] == "dry":
                        in_hum -= air_conditioner["speed"][aircon_status["speed"]]
                        if in_hum < 20:
                            in_hum = 20
                    else:
                        in_hum += 1
                        if in_hum > out_hum:
                            in_hum = out_hum

                elif in_temp != out_temp:
                    in_temp += (out_temp-in_temp)/abs(out_temp-in_temp)
                    if in_temp > out_temp:
                        in_temp = out_temp
                    in_hum += 1
                    if in_hum > out_hum:
                        in_hum = out_hum

                air_clean_status = appliance["inner_room"]["air_cleaner"]

                if air_clean_status["status"] == "ON":
                    air -= air_cleaner["mode"][air_clean_status["mode"]]
                else:
                    air += 1
                    if air > air_condition[self.envir_msg.weather]:
                        air = air_condition[self.envir_msg.weather]

                self.publish_weather()
                print("emit!!!")
                time.sleep(10)
            else:
                time.sleep(1)

    def publish_weather(self):
        if self.get_weather == True:
            global socket_data
            socket_data["message"] = {"environment": environment, "air": air,
                                      "out_temp": out_temp, "out_hum": out_hum,
                                      "in_temp": in_temp, "in_hum": in_hum}

            print("data", socket_data)

            sio.emit('weather_status', json.dumps(socket_data))  # 함수명(토픽) 미정
            print("emit complete")
        else:
            print("can not publish weather")


weather = ''
is_weather = False

sio = socketio.Client()

user_id=1
socket_data = {
    "type": "robot",  # !!!t를 type으로 변경했습니다!!!
    "id": 708002,  # robot ID
    "to": user_id,  # user ID
    "message": {"environment": environment, 
                "air": air,
                "out_temp": out_temp, 
                "out_hum": out_hum,
                "in_temp": in_temp, "in_hum": in_hum
                }
}

init_data = {
    "type": "robot",  # !!!t를 type으로 변경했습니다!!!
    "id": 2708002,  # robot ID
    "to": user_id,  # user ID
    "message": {"environment": environment, 
                "air": air,
                "out_temp": out_temp, 
                "out_hum": out_hum,
                "in_temp": in_temp, "in_hum": in_hum
                }
}

@sio.event
def connect():
    print('서버에 연결되었습니다.')
    sio.emit('init_robot', json.dumps(init_data))
    

@sio.event
def disconnect():
    print('서버와의 연결이 끊어졌습니다.')


@sio.event
def robot_message(data):  # 최초 앱 접속 및 렌더링시 날씨 요청
    global socket_data
    global user_id
    origin = json.loads(data)
    user_id = origin["id"]
    socket_data["to"]=user_id

    msg = origin["message"]
    if msg == "True":
        print("recive data!!")
        # sio.emit('weather_status', json.dumps(socket_data))
        if is_weather :
            weather.publish_weather()
        else:
            sio.emit('weather_status', json.dumps(socket_data))
        print("emit complete")
    print("robot_message_weather!!!!")
    print("user_id", str(socket_data["to"]))


def main(args=None):
    sio.connect('https://j8a708.p.ssafy.io/socket')
    rclpy.init(args=args)
    global weather
    weather = weather_pub()
    is_weather = True
    rclpy.spin(weather)
    weather.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
