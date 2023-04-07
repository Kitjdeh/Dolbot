import socketio
import json

sio = socketio.Client()

dict = {
  "t" : "user",
  "id" : 1,
  "to" : 1,
  "message" : "connect"
}

dict2 = {
  "t" : "user",
  "id" : 1,
  "to" : 1,
  "message" : "hello im user"
}

@sio.event
def connect():
    print('서버에 연결되었습니다.')
    sio.emit('chat_massage', json.dumps(dict))

@sio.event
def disconnect():
    print('서버와의 연결이 끊어졌습니다.')

@sio.event
def chat_message(data):
    print('메시지 수신:', data)

if __name__ == '__main__':
    sio.connect('http://localhost:9998');


    sio.emit('chat_message', json.dumps(dict))

    while True:
        message = input("")
        sio.emit('chat_message', json.dumps(dict2))