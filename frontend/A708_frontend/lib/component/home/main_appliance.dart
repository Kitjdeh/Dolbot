import 'dart:async';
import 'dart:convert';
import 'package:dolbot/component/alert/toast.dart';
import 'package:dolbot/component/home/air_cleaner.dart';
import 'package:dolbot/component/home/aircondition.dart';
import 'package:dolbot/component/home/lamp_tv.dart';
import 'package:dolbot/const/data.dart';
import 'package:dolbot/sockect/sockect.dart';
import 'package:flutter/material.dart';
import 'package:socket_io_client/socket_io_client.dart' as IO;

class MainAppliance extends StatefulWidget {
  const MainAppliance({Key? key}) : super(key: key);

  @override
  State<MainAppliance> createState() => _MainApplianceState();
}

Map<dynamic, dynamic>? _applianceData;

class _MainApplianceState extends State<MainAppliance>
    with TickerProviderStateMixin {
  late final TabController controller;
  StreamSocket streamSocket = StreamSocket();
  IO.Socket? socket;
  List<Widget> tabbarBody = []; // 클래스 멤버 변수로 변경

  // List<Map<String, dynamic>>
  final _appliancestatus = StreamController<Map>();
  String _selectindex = "0";
  Map<dynamic, dynamic>? _cachedApplianceData;

  @override
  void initState() {
    super.initState();
    // Connect to the socket server
    IO.Socket socket = IO.io('http://j8a708.p.ssafy.io:8081',
        IO.OptionBuilder().setTransports(['websocket']).build());
    socket.on('home_status', (data) {
      print('홈 데이터 도착');
      var Data = jsonDecode(data);
      print(Data);
      Map applianceData = Data['message'];
      if (!_appliancestatus.isClosed && mounted && applianceData != null) {
        _appliancestatus.add(applianceData!);
        setState(() {
          _cachedApplianceData = applianceData; // 캐시 업데이트
        });
      };
      toast(context, '집안 데이터가 갱신되었습니다.');
    });
  }

  @override
  void dispose() {
    socket?.disconnect();
    socket = null;
    _appliancestatus.close();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    // 캐시된 데이터 사용
    if (_cachedApplianceData != null) {
      _applianceData = _cachedApplianceData;
    }

    return DefaultTabController(
      length: ROOM.length,
      child: Scaffold(
        appBar: AppBar(
          bottom: PreferredSize(
            preferredSize: Size.fromHeight(1),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.start,
              children: [
                TabBar(
                  indicatorColor: Colors.white,
                  indicatorWeight: 4.0,
                  indicatorSize: TabBarIndicatorSize.tab,
                  isScrollable: true,
                  labelColor: Colors.white,
                  unselectedLabelColor: Colors.grey[400],
                  labelStyle: TextStyle(fontWeight: FontWeight.w700),
                  unselectedLabelStyle: TextStyle(fontWeight: FontWeight.w100),
                  tabs: ROOM
                      .map((e) => Tab(
                              child: Text(
                            e,
                          )))
                      .toList(),
                ),
              ],
            ),
          ),
        ),
        body: StreamBuilder<Map<dynamic, dynamic>>(
            stream: _appliancestatus.stream,
            builder: (context, snapshot) {
              print('tttt;$_applianceData');
              // print('stream$snapshot');
              if (_applianceData != null) {
                print('////');
                return TabBarView(
                  children: _applianceData!.entries
                      .map((entry) => renderCount(entry.key, entry.value))
                      .toList(),
                  // _applianceData!.values
                  //     .map((value) => renderCount((value)))
                  //     .toList(),
                );
                return TabBarView(children: tabbarBody);
              } else {
                return TabBarView(
                    // physics:,
                    children: ROOM
                        .map((e) => Tab(
                                child: Text(
                              e,
                            )))
                        .toList());
              }
            }),
      ),
    );
  }

  // children: e!.entries
  //     .map((k, v) =>
  // renderContainer(title: k, img: e.img, onoff: v.status, index: 1))
  //     .toList(),
  // List<MapEntry<dynamic, dynamic>> S = e.entries.toList();
  Widget renderCount(String key, Map<String, dynamic> data) {
    print('data데이터${data}e');
    String roomname = key;
    return GridView.count(
      crossAxisCount: 2,
      childAspectRatio: 1.0,
      // shrinkWrap: true,
      // scrollDirection: Axis.vertical,
      children: data!.entries
          .map((MapEntry<String, dynamic> entry) => renderContainer(
                room: roomname,
                title: entry.key,
                img: entry.key,
                onoff: entry.value['status'],
                index: 1,
                mode: entry.value['mode'] != null ? entry.value['mode'] : '',
                speed: entry.value['speed'] != null ? entry.value['speed'] : '',
                target:
                    entry.value['target'] != null ? entry.value['target'] : 0,
              ))
          .toList(),
    );
  }
  // else {
  //   return GridView.count(
  //     crossAxisCount: 2,
  //     children: ROOMINFO
  //         .map((e) => renderContainer(
  //             title: e.title,
  //             img: e.img,
  //             onoff: e.ONOFF,
  //             index: e.index))
  //         .toList(),
  //   );
  // }

  Widget renderContainer({
    required String title,
    required String img,
    required String onoff,
    required int index,
    required String room,
    int? temperature,
    required String mode,
    required int target,
    required String speed,

    // double? height,f
  }) {
    return Padding(
        padding: const EdgeInsets.all(8.0),
        child: GestureDetector(
          onTap: () {
            setState(() {
              _selectindex = title;
              if (onoff == 'ON') {
                if (_selectindex == 'air_conditioner') {
                  showModalBottomSheet(
                      context: context,
                      builder: (_) {
                        return AirCondition(
                          room: room,
                          mode: mode,
                          target: target,
                          speed: speed,
                        );
                      });
                } else if (_selectindex == 'air_cleaner') {
                  {
                    showModalBottomSheet(
                        context: context,
                        builder: (_) {
                          return AirCleaner(
                            room: room,
                            mode: mode,
                          );
                        });
                  }
                }
              }
            });
          },
          child: Container(
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(16),
                color: onoff == 'ON' ? Colors.blue[100] : Colors.grey[300],
              ),
              // height: height == null ? 300 : height,

              child: Center(
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    Row(
                      mainAxisAlignment: MainAxisAlignment.end,
                      children: [
                        SwitchStream(
                          onoff: onoff,
                          title: title,
                          img: img,
                          index: index,
                          room: room,
                          mode: mode,
                          speed: speed,
                          target: target,
                        )
                      ],
                    ),
                    Container(
                        decoration: BoxDecoration(
                            color: Colors.white,
                            borderRadius: BorderRadius.circular(100)),
                        child: SizedBox(
                            height: 110,
                            width: 110,
                            child: Image.asset(
                              'asset/img/$img.png',
                            ))),
                    Padding(
                      padding: const EdgeInsets.symmetric(horizontal: 8.0),
                      child: Row(
                        children: [
                          Text(
                            '$title',
                            style: TextStyle(
                                color: Colors.black,
                                fontWeight: FontWeight.w700,
                                fontSize: 20),
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              )),
        ));
  }

  // Widget selectCount() {
  //   return AirCondition(
  //     room: room,
  //     mode:mode,
  //     target: target,
  //     speed: speed,
  //   );
  // }

  Future<bool> getNumber(onoff) async {
    await Future.delayed(Duration(seconds: 3));
    final bool result = onoff == false ? true : false;
    print('요청');
    // throw Exception('에러 발생');
    return result;
  }
}

class SwitchStream extends StatelessWidget {
  final String onoff;
  final String title;
  final String img;
  final int index;
  final String room;
  final String mode;
  final String speed;
  final int target;
  const SwitchStream(
      {required this.onoff,
      required this.title,
      required this.img,
      required this.index,
      required this.room,
      required this.speed,
      required this.mode,
      required this.target,
      Key? key})
      : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
        child: Switch(
      value: onoff == 'ON' ? true : false,
      onChanged: (value) {
        value = !value;
        print('$title ${value.toString()}');
        if (title == 'air_cleaner' && value == false) {
          showModalBottomSheet(
              context: context,
              builder: (_) {
                return AirCleaner(
                  room: room,
                  mode: mode,
                );
              });
        } else if (title == 'air_conditioner' && value == false) {
          showModalBottomSheet(
              context: context,
              builder: (_) {
                return AirCondition(
                  room: room,
                  mode: mode,
                  target: target,
                  speed: speed,
                );
              });
        } else {
          Map<String, dynamic> A = {
            'type': 'user',
            'id': 1,
            'to': 708001,
            'message': {
              'room': room,
              'device': title,
              'status': onoff == 'ON' ? 'OFF' : 'ON',
            }
          };
          String test = jsonEncode(A);
          SendMessage(test);
          print(test);
        }

        // setState(() {
        //   onoff = value;
        // });
      },
      activeTrackColor: Colors.blue,
    ));
  }
}

// class RoomInfo {
//   final String title;
//   final bool ONOFF;
//   final int? temperature;
//   final int index;
//   final String img;
//   const RoomInfo(
//       {required this.title,
//       required this.ONOFF,
//       required this.index,
//       required this.img,
//       this.temperature});
// }
//
// const ROOMINFO = [
//   RoomInfo(
//       title: '에어컨',
//       img: 'asset/img/air_conditioner.png',
//       ONOFF: true,
//       temperature: 30,
//       index: 0),
//   RoomInfo(
//       title: '공기청정기', img: 'asset/img/air_cleaner.png', ONOFF: false, index: 1),
//   RoomInfo(title: '조명', img: 'asset/img/light.png', ONOFF: true, index: 2),
//   RoomInfo(title: 'TV', img: 'asset/img/tv.png', ONOFF: true, index: 3),
// ];
