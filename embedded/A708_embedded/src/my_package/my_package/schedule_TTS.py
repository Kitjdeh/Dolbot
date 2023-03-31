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

from gtts import gTTS
from playsound import playsound

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
        
class Schedule(Node):
   
    def __init__(self):
        super().__init__(node_name='Schedule')
        print("시작")

        self.res = requests.get('http://3.36.67.119:8080/api/v1/schedule-info/1?localDate='+'2023-03-23')
        self.res = self.res.json()
        print(self.res)

        thread = threading.Thread(target=self.tts)
        thread.daemon = True
        thread.start()

    def tts(self):

        idx=0
        while True:

            if idx==len(self.res):
                break

            now_time = time.time()
            now_time=localtime(now_time)
            print(now_time)

            schedule_time = self.res[idx]['scheduleTime']
            print(schedule_time)
            schedule_time_sec = schedule_time // 1000
            schedule_datetime = datetime.datetime.fromtimestamp(schedule_time_sec)
            hour, minutes = schedule_datetime.hour, schedule_datetime.minute

            print(now_time.tm_hour, now_time.tm_min, hour, minutes)
            if now_time.tm_hour == hour and now_time.tm_min == minutes:
                print(self.res[idx]['content'])
                speak(self.res[idx]['content'])
                idx+=1
            
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

    rclpy.init(args=args)
    schedule = Schedule()
    rclpy.spin(schedule)

if __name__ == '__main__':
    main()

