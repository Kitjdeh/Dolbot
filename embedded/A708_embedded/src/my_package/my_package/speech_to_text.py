from __future__ import division

import re
import sys

from google.cloud import speech

import pyaudio
from six.moves import queue
from gtts import gTTS
from playsound import playsound
import threading
import time

import os
from . import iot_udp

os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "C:/TTS/vibrant-period-381607-92ab31325bad.json"

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

stt_flag=False

mode = ['cool', 'mid']
cmd = {'room': 'living_room', 'device': 'tv', 'status': 'OFF','mode': '','speed':'mid','target':18}

# STT
def speechToText():
    language_code = "ko-KR"  # a BCP-47 language tag

    client = speech.SpeechClient()
    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code,
    )

    streaming_config = speech.StreamingRecognitionConfig(
        config=config, interim_results=True
    )

    with MicrophoneStream(RATE, CHUNK) as stream:
        audio_generator = stream.generator()
        requests = (
            speech.StreamingRecognizeRequest(audio_content=content)
            for content in audio_generator
        )

        responses = client.streaming_recognize(streaming_config, requests)

        # Now, put the transcription responses to use.
        listen_print_loop(responses)

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

# 대답
def answer(input_text):
     global cmd
     answer_text=''

     if '돌쇠야' == input_text:
          speak('네 어르신 말씀하세요.')
          # speak('안녕하세요. 어르신을 케어하는 로봇 돌봇입니다. 무엇을 도와드릴까요?')
          return

     elif '돌쇠야' in input_text and "안녕" in input_text:
          speak('네 어르신 안녕하세요')

     elif '어디' in input_text:
          speak('네 어르신 저 여기에 있어요.')

     else:
          room=''
          appliance=''
          status=''

          room_text=''
          appliance_text=''

          if '거실' in input_text:
               room='living_room'
               room_text='거실'
               if '조명' in input_text or '불' in input_text:
                    appliance='light'
                    appliance_text='조명'
               elif '에어컨' in input_text:
                    appliance='air_conditioner'
                    appliance_text='에어컨'
               elif 'TV' in input_text or 'tv' in input_text:
                    appliance='tv'
                    appliance_text='티비'

          elif '안방' in input_text:
               room='inner_room'
               room_text='안방'
               if '조명' in input_text or '불' in input_text:
                    appliance='light'
                    appliance_text='조명'
               elif '에어컨' in input_text:
                    appliance='air_conditioner'
                    appliance_text='에어컨'
               elif '공기' in input_text and '청정' in input_text:
                    appliance='air_cleaner'
                    appliance_text='공기청정기'
               
          elif '서재' in input_text:
               room='library'
               room_text='서재'
               if '조명' in input_text or '불' in input_text:
                    appliance='light'
                    appliance_text='조명'
               elif '공기' in input_text and '청정' in input_text:
                    appliance='air_cleaner'
                    appliance_text='공기청정기'

          elif '작은' in input_text and '방' in input_text:
               room='small_room'
               room_text='작은방'
               if '조명' in input_text or '불' in input_text:
                    appliance='light'
                    appliance_text='조명'
               elif '에어컨' in input_text:
                    appliance='air_conditioner'
                    appliance_text='에어컨'
               elif 'TV' in input_text or 'tv' in input_text:
                    appliance='tv'
                    appliance_text='티비'
     
          elif '화장실' in input_text:
               room='toilet'
               room_text='화장실'
               if '조명' in input_text or '불' in input_text:
                    appliance='light'
                    appliance_text='조명'

          if '켜' in input_text:
               status='ON'
          elif '꺼' in input_text:
               status='OFF'
          
          if room!='' and appliance!='' and status!='':
               speak('네')
               speak_text=room_text+'의 '+appliance_text
               if appliance_text=='공기청정기' or appliance_text=='티비':
                    speak_text+='를 '
               else:
                    speak_text+='을 '
               if status=='ON':
                    speak_text+='켜드릴게요.'
               else:
                    speak_text+='꺼드릴게요.'
               speak(speak_text)
               print(room, appliance, status)
               
               cmd['room'] = room
               cmd['device'] = appliance
               cmd['status'] = status

               if appliance=='air_conditioner':
                    cmd['mode'] = mode[0]
               elif appliance == 'air_cleaner':
                    cmd['mode'] = mode[1]

               iot_udp.iot.device_control(cmd)
               
          else:
               speak('방 이름과 가전기기를 함께 말해주세요.')

class MicrophoneStream(object):

    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk

        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self._rate,
            input=True,
            frames_per_buffer=self._chunk,
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
          
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b"".join(data)

def listen_print_loop(responses):

    global stt_flag
    num_chars_printed = 0
    for response in responses:
        if not response.results:
            continue

        result = response.results[0]
        if not result.alternatives:
            continue

        transcript = result.alternatives[0].transcript

        overwrite_chars = " " * (num_chars_printed - len(transcript))

        if not result.is_final:
            sys.stdout.write(transcript + overwrite_chars + "\r")
            sys.stdout.flush()

            num_chars_printed = len(transcript)

        else:
            print("[USER] "+transcript)

            if stt_flag==False and '돌쇠' in transcript:
                # playsound('start.mp3')
                speak('안녕하세요. 어르신을 케어하는 로봇 돌봇입니다. 무엇을 도와드릴까요?')
                stt_flag=True
            
            elif stt_flag==True:
                answer(transcript)


            if re.search(r"\b(종료)\b", transcript, re.I):
                print("Exiting..")
                break

            num_chars_printed = 0

