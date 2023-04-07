import socketio
import eventlet
import json


users = {};
robots = {};

sio = socketio.Server()
app = socketio.WSGIApp(sio)

@sio.event
def connect(sid, environ):
    print(type(sid))
    print('새로운 클라이언트 접속:', sid)

@sio.event
def disconnect(sid):
    print('클라이언트가 연결 해제:', sid)

@sio.event
def chat_message(sid, data):
    dict = json.loads(data);
    # print(data);
    # print(dict);
    print("users");
    print(users);
    print("robots");
    print(robots)
    if dict['t'] == 'robot':
        if dict['message'] == 'connect':
            print(str(dict['t']) + "번 로봇 참여");
            robots[str(dict['id'])] = sid;
            sio.emit('chat_message', "환영합니다 " + str(dict["id"]) + "번 로봇", to=sid)
        else:
            sio.emit('chat_message', data, to=users[str(dict['to'])]);
    else: 
        if dict['message'] == 'connect':
            print(str(dict['t']) + "번 어플 참여");
            users[str(dict['id'])] = sid;
            sio.emit('chat_message', "환영합니다 " + str(dict["id"]) + "번 어플", to=sid)
        else:
            sio.emit('chat_message', data, to=robots[str(dict['to'])]);
    # print('메시지 수신:', data)
    # sio.emit('chat_message', data)

if __name__ == '__main__':
    eventlet.wsgi.server(eventlet.listen(('localhost', 5000)), app)