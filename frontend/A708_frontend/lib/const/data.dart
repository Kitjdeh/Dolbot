import 'package:flutter/material.dart';

const ROOM = ['거실', '안방', '서재', '작은방','화장실', '현관'];
const rainbowColors = [
  Colors.red,
  Colors.orange,
  Colors.yellow,
  Colors.green,
  Colors.blue,
  Colors.indigo,
  Colors.purple
];
const APPLIANCE = ['lamp', 'TV', '공기청정기', '에어컨'];
const List<Map<String, dynamic>> DATABASE = [
  {
    'title': '안방',
    'appliance': ['lamp', 'TV', 'aircleaner', 'airconditioner'],
  },
  {
    'title': '거실',
    'appliance': ['lamp']
  },
  {
    'title': '화장실',
    'appliance': ['lamp']
  },
  {
    'title': '작은방',
    'appliance': ['lamp', 'airconditioner']
  },
];


class LogInfo {
  final int endtime;
  final int index;
  final String location;
  final String content;

  const LogInfo({
    required this.index,
    required this.location,
    required this.content,
    required this.endtime,
  });
}

const LOGINFO = [
  LogInfo(location: '거실', content: '에어컨을 켰습니다..', endtime: 10, index: 0),
  LogInfo(location: '화장실', content: '불이 꺼졌습니다.', endtime: 50, index: 0),
  LogInfo(location: '거실', content: '불이 꺼졌습니다.', endtime: 60, index: 0),
  LogInfo(location: '일정', content: '병원 예약', endtime: 110, index: 2),
  LogInfo(location: '비상', content: '김원혁님이 쓰러졌습니다..', endtime: 150, index: 1),
  LogInfo(location: '일정', content: '점심약 복용', endtime: 210, index: 2),
  LogInfo(location: '거실', content: '에어컨이 꺼졌습니다.', endtime: 260, index: 0),
];



Map socket_data = {
"type": "robot",
"id": 1,
"to": 1,
'message': {
"living_room": {
"light": {"status": "OFF", "pose": [-5.54, 5.31]},
"air_conditioner": {"status": "OFF", "mode": "cool", "speed": "mid", "target": 23, "pose": [-2.68, 4.38]},
"tv": {"status": "OFF", "pose": [-5.54, 5.31]},
},
"inner_room": {
"light": {"status": "OFF", "pose": [0, 0]},
"air_conditioner": {"status": "OFF", "mode": "cool", "speed": "mid", "pose": [-10.0, 4.79]},
"air_cleaner": {"status": "OFF", "mode": "high", "pose": [-12.29, 6.20]},
},
"library": {
"light": {"status": "OFF", "pose": [0, 0]},
"air_cleaner": {"status": "OFF", "pose": [-4.04, 12.02]},
},
"small_room": {
"light": {"status": "OFF", "pose": [0, 0]},
"air_conditioner": {"status": "OFF", "mode": "cool", "speed": "mid", "pose": [0, 0]},
"tv": {"status": "OFF", "pose": [-3.64, 14.97]},
},
"toilet": {
"light": {"status": "OFF", "pose": [0, 0]},
},
"entrance": {
"light": {"status": "OFF", "pose": [0, 0]},
},
}

};
