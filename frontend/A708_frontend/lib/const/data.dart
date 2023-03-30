import 'package:flutter/material.dart';

const ROOM = ['안방', '화장실', '거실', '작은방','거실', '작은방'];
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
