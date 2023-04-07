import 'dart:convert';
import 'dart:typed_data';

import 'package:dolbot/component/alert/toast.dart';
import 'package:dolbot/component/login/kakao_login.dart';
import 'package:dolbot/const/tabs.dart';
import 'package:dolbot/screen/calendar_screen.dart';
import 'package:dolbot/screen/cctv_screen.dart';
import 'package:dolbot/screen/home_screen.dart';
import 'package:dolbot/screen/log_screen.dart';
import 'package:dolbot/screen/main/main_screen.dart';
import 'package:dolbot/screen/menu_screen.dart';
import 'package:dolbot/screen/splash_screen.dart';
import 'package:flutter/material.dart';
import 'package:kakao_flutter_sdk/kakao_flutter_sdk_user.dart';
import 'package:socket_io_client/socket_io_client.dart' as IO;

import '../../restapi/user_rest_api.dart';

class MainScreen extends StatefulWidget {
  String? username;
  int? kakaoID;
  String? profile;
  String? homename;
  int? homeId;
  int? userId;
  Image? RosImage;
  MainScreen(
      {this.homename,
      this.homeId,
      this.username,
      this.kakaoID,
      this.profile,
      this.userId,
      this.RosImage,
      Key? key})
      : super(key: key);

  @override
  State<MainScreen> createState() => _MainScreenState();
}

class _MainScreenState extends State<MainScreen> with TickerProviderStateMixin {
  late final TabController controller;
  int? out_temp;
  int? in_temp;
  int? in_hum;
  int? out_hum;
  int? air;
  String? environment;
  Map<dynamic, dynamic>? ApplianceData;
  IO.Socket socket = IO.io(
      'http://j8a708.p.ssafy.io:8081',
      IO.OptionBuilder()
          .setTransports(['websocket'])
          .setTimeout(1000000000000)
          .build());

  @override
  void initState() {
    // TODO: implement initState
    super.initState();

    controller = TabController(
      length: TABS.length,
      vsync: this,
    );
    controller.addListener(() {
      setState(() {});
    });
    Map<String, dynamic> msg = {
      'type': 'user',
      'id': 4,
      'to': 708002,
      'message': 'True'
    };
    socket.onConnect((_) {
      String message = jsonEncode(msg);
      socket.emit('init_user', message);
      print('connect');
      socket.emit('user_message', message);
      print(message);
    });
    socket.on('weather_status', (data) {
      var weatherData = jsonDecode(data);
      var message = weatherData['message'];
      // print('파싱$weatherData //// $out_temp ///');
      Map<String, dynamic> weatherMap = json.decode(data);
      // print(weatherMap);
      if (message['out_temp'] != null && mounted) {
        toast(context, '현재 날씨');
        setState(() {
          out_temp = message['out_temp'];
          in_temp = message['in_temp'];
          out_hum = message['out_hum'];
          in_hum = message['in_hum'];
          environment = message['environment'];
          air = message['air'];
        });
      }
      // print('out_hum$out_hum');
    });
    socket.on('home_status', (data) {
      print('홈 데이터 도착');
      var Data = jsonDecode(data);
      Map applianceData = Data['message'];
      if (mounted && applianceData != null) {
        setState(() {
          ApplianceData = applianceData; // 캐시 업데이트
        });
      }
      ;
      toast(context, '집안 데이터가 갱신되었습니다.');
    });

    socket.on('emergency', (data) async {
      ;
      var Data = jsonDecode(data);
      String message = Data['message'];
      // String message = json.decode(Emergency_message);
      emergency(context, message);
    });
    Map<String, dynamic> A = {
      'type': 'user',
      'id': 4,
      'to': 708002,
      'message': '',
    };
    String message = jsonEncode(A);
    print('cctvon$message');
    socket.emit('cctv_on', message);
    // });
    // Listen for weather updates
    // }

    socket.on('video', (data) {
      print('video');
      // if (mounted) {
      var Data = jsonDecode(data);
      var ImageData = (Data['data'] as List<dynamic>).cast<int>();
      Uint8List _byte = Uint8List.fromList(ImageData);
      print('coming');
      if (ImageData != null) {
        setState(() {
          widget.RosImage =
              Image.memory(_byte, width: 320, height: 240, fit: BoxFit.fill);
        });
      }
      // }
    });
  }
  // late Future<Robot>? Robot;
  // late final Future<Log>? LogData = getLog(widget.selectedDate);

  // Connect to the socket server

  @override
  Widget build(BuildContext context) {
    // print('main 로봇${Robot}');
    return Scaffold(
      backgroundColor: Colors.blue[0],
      appBar: AppBar(
        backgroundColor: Colors.white,
        title: Padding(
          padding: EdgeInsets.only(bottom: 8.0),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.end,
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              Row(
                children: [
                  ClipRRect(
                    borderRadius: BorderRadius.circular(30),
                    child: SizedBox(
                        height: 30,
                        child: Image.network(widget?.profile ?? '')),
                  ),
                  Text(
                    '반가워요 ${widget?.username}님',
                    style: TextStyle(fontSize: 20, color: Colors.black),
                  ),
                ],
              ),
              Text('${widget?.homename}님 의 홈',
                  style: TextStyle(fontSize: 12, color: Colors.black))
            ],
          ),
        ),
      ),
      body: TabBarView(
        physics: NeverScrollableScrollPhysics(),
        // clipBehavior: ,
        controller: controller,
        children: [
          HomeScreen(
            air: air,
            aplicancestatus: ApplianceData,
            socket: socket,
            out_hum: out_hum,
            in_hum: in_hum,
            in_temp: in_temp,
            out_temp: out_temp,
            environment: environment,
          ),
          CalendarScreen(),
          LogScreen(),
          CctvScreen(
            RosImage: widget.RosImage,
            user: widget.userId ?? 0,
            robotnumber: widget.homeId ?? 0,
          ),
          MenuScreen(
              username: widget.username ?? '',
              userid: widget.userId ?? 0,
              mainhomeId: widget.homeId ?? 0,
              kakaoId: widget.kakaoID ?? 0,
              profile: widget.profile ?? '')
        ],
      ),
      bottomNavigationBar: BottomNavigationBar(
        selectedItemColor: Colors.blue,
        unselectedItemColor: Colors.grey,
        showSelectedLabels: true,
        showUnselectedLabels: true,
        currentIndex: controller.index,
        type: BottomNavigationBarType.shifting,
        onTap: (index) {
          controller.animateTo(index);
        },
        items: TABS
            .map(
              (e) =>
                  BottomNavigationBarItem(icon: Icon(e.icon), label: e.label),
            )
            .toList(),
      ),
    );
  }
}

class Robot {
  final int? userHomeId;
  final int? userId;
  final int? homeId;
  final String? nickname;
  final int? robotNumber;
  final bool? alarm;

  Robot(
      {this.userHomeId,
      this.userId,
      this.homeId,
      this.nickname,
      this.robotNumber,
      this.alarm});

  factory Robot.fromJson(Map<String, dynamic> json) {
    // print(json["scheduleTime"].runtimeType);
    // print(json["content"].runtimeType);
    // print(json["startDate"].runtimeType);
    return Robot(
        userHomeId: json["userHomeId"],
        userId: json["userId"],
        homeId: json["homeId"],
        nickname: json["nickname"],
        robotNumber: json["robotNumber"],
        alarm: json["alarm"]);
  }
}
