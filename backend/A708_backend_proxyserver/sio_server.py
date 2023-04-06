import socketio
import eventlet
import json

#로봇 정보 등록에서 target 리스트 안만듬 대신 유저가 on 할때 없으면 만듬

users = {}
robots = {}
target={}

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
    uId = dict['to']
    sio.emit('weather_status', data, to=users[str(uId)])

# 기기의 제어를 로봇에게 요청하는 함수 FE -> EM
@sio.event
def appliance_status(sid, data):
    print("appliance_status")
    dict = json.loads(data)
    print(data)
    rId = dict['to']
    sio.emit('appliance_status', data, to=robots[str(rId + 1000000)])

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
    rId = dict['to']
    sio.emit('schedule',data, to=robots[str(rId + 3000000)])

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

#유저가 로봇에게 보내는 메시지, 기기 정보 요청
@sio.event
def user_message(sid, data):  # sid는 socket의 id
    print("user_message")
    print(data)
    dict = json.loads(data)
    rId = dict['to']
    
    for i in range(4):
        sio.emit('robot_message', data, to=robots[str(rId + (i * 1000000))])

# 유저가 CCTV 화면을 킴
@sio.event
def cctv_on(sid,data):
    print("cctv_on")
    print(data)
    dict = json.loads(data)
    robotId = str(dict['to'])
    userId = str(dict['id'])
    if robotId not in target:
        print("등록" + robotId)
        target[robotId] = []
    if userId in target[robotId]:
        return
    target[robotId].append(userId)
    print(target)

# 유저가 CCTV 화면을 끔
@sio.event
def cctv_off(sid,data):
    print("cctv_off")
    print(data)
    dict = json.loads(data)
    robotId = str(dict['to'])
    userId = str(dict['id']) 
    target[robotId].remove(userId)

# 로봇이 CCTV를 킨 유저들에게 영상을 송출
@sio.event
def video(sid,data):    
    print("video") 
    dict = json.loads(data)
    print(target[str(dict['id'])])
    if str(dict['id']) not in target:
        return
    for i in target[str(dict['id'])]:
        users[i]
        sio.emit('video',data, to=users[i])

#cctv 조작
@sio.event
def cctv(sid,data):
    print("cctv")
    dict = json.loads(data)
    print(data)
    sio.emit('cctv',data, to=robots[str(dict['to'])])

if __name__ == '__main__':
    eventlet.wsgi.server(eventlet.listen(('', 8081)), app)