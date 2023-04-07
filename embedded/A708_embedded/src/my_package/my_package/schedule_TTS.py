# 이 노드가 가장 먼저 실행되어야 logListId가 생성되고, 생성된 logListId를 이용해서 로그 POST 요청 가능 

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

import time
from time import localtime
import requests
import json
import base64
import socketio
import os
import threading
import datetime
import socketio

from gtts import gTTS
from playsound import playsound
# from . import iot_udp

sio = socketio.Client()
# sio=iot_udp.sio
schedule_info={}
user_id=0

# TTS
def speak(text):
     if text=='':
        return
     print('[AI] '+ text)
     tts = gTTS(text=text, lang='ko')
     filename='voice.mp3'
     tts.save(filename)
     playsound(filename)

     if os.path.exists(filename):
        os.remove(filename)

socket_data = {
    "type": "robot", # !!!t를 type으로 변경했습니다!!!
    "id": 708002,  # robot ID
    "to": user_id,  # user ID
    'message': "robot"
}      

init_data = {
    "type": "robot", # !!!t를 type으로 변경했습니다!!!
    "id": 3708002,  # robot ID
    "to": user_id,  # user ID
    'message': "robot"
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
def schedule(data):
    global schedule_info
    global user_id
    print('일정 변경 및 삭제 메세지 수신: ', data)
    data = json.loads(data)
    user_id = data["id"]
    t = time.time()

    month = localtime(t).tm_mon
    day = localtime(t).tm_mday

    if month<10: 
        month='0'+str(month)
    if day<10:
        day='0'+str(day)

    today = str(localtime(t).tm_year)+"-"+str(month)+"-"+str(day)

    # res = requests.get('https://j8a708.p.ssafy.io/api/v1/schedule-info/1?localDate='+'2023-03-23')
    res = requests.get('https://j8a708.p.ssafy.io/api/v1/schedule-info/1?localDate='+today)
    res = res.json()
    schedule_info = res
    print(res) 

class Schedule(Node):
   
    def __init__(self):
        super().__init__(node_name='Schedule')
        print("시작")

        t = time.time()
        month = localtime(t).tm_mon
        day = localtime(t).tm_mday

        if month<10: 
            month='0'+str(month)
        if day<10:
            day='0'+str(day)

        today = str(localtime(t).tm_year)+"-"+str(month)+"-"+str(day)

        # self.res = requests.get('https://j8a708.p.ssafy.io/api/v1/schedule-info/1?localDate='+'2023-03-23')
        self.res = requests.get('https://j8a708.p.ssafy.io/api/v1/schedule-info/1?localDate='+today)
        self.res = self.res.json()
        print(self.res)

        global schedule_info 
        schedule_info = self.res

        thread = threading.Thread(target=self.tts)
        thread.daemon = True
        thread.start()

    def tts(self):
        global schedule_info
        idx=0
        while True:
            # print(schedule_info)
            if idx>=len(schedule_info):
                break

            now_time = time.time()
            now_time=localtime(now_time)
            print(now_time)

            schedule_time = schedule_info[idx]['scheduleTime']
            print(schedule_time)
            schedule_time_sec = schedule_time // 1000
            schedule_datetime = datetime.datetime.fromtimestamp(schedule_time_sec)
            hour, minutes = schedule_datetime.hour, schedule_datetime.minute

            print(now_time.tm_hour, now_time.tm_min, hour, minutes)
            if now_time.tm_hour == hour and now_time.tm_min == minutes:
                # on_speak=True, on_control, on_return, on_cctv => False 
                print(schedule_info[idx]['content'])
                speak("어르신, 오늘의 일정은 "+schedule_info[idx]['content']+"입니다.")
                # on_speak=False
                idx+=1
            
            if now_time.tm_hour > hour:
                idx+=1
                continue

            if now_time.tm_hour == hour and now_time.tm_min>minutes:
                idx+=1
                continue

            time.sleep(10)

        # for r in self.res:
        #     schedule_time = r['scheduleTime']
        #     print(schedule_time)
        #     schedule_time_sec = schedule_time // 1000
        #     schedule_datetime = datetime.datetime.fromtimestamp(schedule_time_sec)
        #     hour, minutes = schedule_datetime.hour, schedule_datetime.minute
        #     print(schedule_datetime)
        #     print(hour)
        #     print(minutes)
        #     print(r['content'])
        #     speak(r['content'])
        #     time.sleep(20)

def main(args=None):
    global sio
    sio.connect('https://j8a708.p.ssafy.io/socket')
    rclpy.init(args=args)
    schedule = Schedule()
    rclpy.spin(schedule)

if __name__ == '__main__':
    main()