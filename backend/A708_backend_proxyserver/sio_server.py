import socketio
import eventlet
import json

users = {};
robots = {};

sio = socketio.Server()
app = socketio.WSGIApp(sio)

@sio.event
def connect(sid, environ):
    print('새로운 클라이언트 접속:', sid)

@sio.event
def disconnect(sid):
    print('클라이언트가 연결 해제:', sid)

@sio.event
def init_robot(sid, data):
    print(data)
    dict = json.loads(data)
    robots[str(dict['id'])] = sid
    print('robot-sid 매핑:', dict['id'], robots[str(dict['id'])])

@sio.event
def init_user(sid, data):
    print(data)
    dict = json.loads(data)
    users[str(dict['id'])] = sid
    print('user-sid 매핑:', dict['id'], sid)

@sio.event
def robot_message(sid, data):  # sid는 socket의 id
    print(data)
    dict = json.loads(data)

    print(str(dict['id']) + '번 로봇 참여')
    robots[str(dict['id'])] = sid  # robot ID에 소켓 ID 매핑
    #sio.emit('user_message', "환영합다. 이 메시지는 "+str(dict["id"])+"번 로봇이 유저" + str(dict['to']) +"에게 보낸 메시지입니다.", to=users[str(dict['to'])])
    sio.emit('user_message', data, to=users[str(dict['to'])])
@sio.event
def user_message(sid, data):  # sid는 socket의 id
    print(data)
    dict = json.loads(data)
    print(str(dict['id']) + '번 유저 참여')
    users[str(dict['id'])] = sid  # user ID에 소켓 ID 매핑
    #sio.emit('robot_message', "환영합니다. 이 메시지는 "+str(dict["id"])+"번 유저가 로봇" + str(dict['to']) +"에게 보낸 메시지입니다.", to=robots[str(dict['to'])])
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
    # print('메시지 수신:', data)
    # sio.emit('chat_message', data)

if __name__ == '__main__':
    eventlet.wsgi.server(eventlet.listen(('', 8081)), app)
