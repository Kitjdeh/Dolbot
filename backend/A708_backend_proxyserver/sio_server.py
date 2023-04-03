import socketio
import eventlet
import json

users = {};
robots = {};

sio = socketio.Server()
app = socketio.WSGIApp(sio)

# 누군가 연결했을 때 호출
@sio.event
def connect(sid, environ):
    print('새로운 클라이언트 접속:', sid)

# 누군가 연결을 해제 했을 때 호출
@sio.event
def disconnect(sid):
    print('클라이언트가 연결 해제:', sid)

# 로봇의 정보를 등록하는 함수
@sio.event
def init_robot(sid, data):
    print(data)
    dict = json.loads(data)
    robots[str(dict['id'])] = sid
    print('robot-sid 매핑:', dict['id'], robots[str(dict['id'])])

# 유저의 정보를 등록하는 함수
@sio.event
def init_user(sid, data):
    print(data)
    dict = json.loads(data)
    users[str(dict['id'])] = sid
    print('user-sid 매핑:', dict['id'], sid)

# 날씨의 상태를 유저에게 전달하는 함수 EM -> FE
@sio.event
def weather_status(sid, data):
    print("weather_status")
    dict = json.loads(data)
    print(data)
    sio.emit('weather_status', data, to=users[str(dict['to'])])

# 기기의 제어를 로봇에게 요청하는 함수 FE -> EM
@sio.event
def appliance_status(sid, data):
    print("appliance_status")
    dict = json.loads(data)
    print(data)
    sio.emit('appliance_status', data, to=robots[str(dict['to'])])

# 집의 기기들의 상태를 유저에게 전달하는 함수 EM -> FE
@sio.event
def home_status(sid, data):
    print("home_status")
    dict = json.loads(data)
    print(data)
    sio.emit('home_status', data, to=users[str(dict['to'])])

# 스케줄의 변동을 로봇에게 알리는 함수 FE -> EM
@sio.event
def schedule(sid, data):
    print("schedule")
    dict = json.loads(data)
    print(data)
    sio.emit('schedule',data, to=robots[str(dict['to'])])

# 제어 일정 응급에 대한 알림 메시지 EM -> FE
@sio.event
def toast(sid, data):
    print("toast")
    dict = json.loads(data)
    print(data)
    sio.emit('toast',data,to=users[str(dict['to'])])


#로봇이 유저에게 보내는 메시지
@sio.event
def robot_message(sid, data):  # sid는 socket의 id
    print("robot_message")
    print(data)
    dict = json.loads(data)

    robots[str(dict['id'])] = sid
    sio.emit('user_message', data, to=users[str(dict['to'])])

#유저가 로봇에게 보내는 메시지
@sio.event
def user_message(sid, data):  # sid는 socket의 id
    print("user_message")
    print(data)
    dict = json.loads(data)
    
    users[str(dict['id'])] = sid
    sio.emit('robot_message', data, to=robots[str(dict['to'])])

@sio.event
def chat_message(sid, data):
    dict = json.loads(data)

    if dict['type'] == 'robot':
        if dict['message'] == 'connect':
            print(str(dict['type']) + "번 로봇 참여")
            robots[str(dict['id'])] = sid
            sio.emit('chat_message', "환영합니다 " + str(dict["id"]) + "번 로봇", to=sid)
        else:
            sio.emit('chat_message', data, to=users[str(dict['to'])])
    else: 
        if dict['message'] == 'connect':
            print(str(dict['type']) + "번 어플 참여")
            users[str(dict['id'])] = sid
            sio.emit('chat_message', "환영합니다 " + str(dict["id"]) + "번 어플", to=sid)
        else:
            sio.emit('chat_message', data, to=robots[str(dict['to'])])

target={}
@sio.event
def targeting(sid,data):
    dict = json.loads(data)
    robotSid = robots[str(dict['id'])]
    userSid = users[str(dict['to'])] 
    target[robotSid] = userSid 

@sio.event
def video(sid,data): 
    sio.emit('video',data, to=target[str(sid)])

if __name__ == '__main__':
    eventlet.wsgi.server(eventlet.listen(('', 8081)), app)
