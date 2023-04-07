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
  int? air;
  Map<dynamic, dynamic>? aplicancestatus;
  MainAppliance({this.aplicancestatus, this.air, Key? key}) : super(key: key);

  @override
  State<MainAppliance> createState() => _MainApplianceState();
}

Map<dynamic, dynamic>? _applianceData;

class _MainApplianceState extends State<MainAppliance>
    with TickerProviderStateMixin {
  late final TabController controller;
  // StreamSocket streamSocket = StreamSocket();
  // IO.Socket? socket;
  List<Widget> tabbarBody = []; // 클래스 멤버 변수로 변경

  // List<Map<String, dynamic>>
  final _appliancestatus = StreamController<Map>();
  String _selectindex = "0";
  Map<dynamic, dynamic>? _cachedApplianceData;

  // @override
  // void dispose() {
  //   if (socket != null) {
  //     socket!.disconnect();
  //   }
  //   _appliancestatus.close();
  //   super.dispose();
  // }

  @override
  Widget build(BuildContext context) {
    // 캐시된 데이터 사용
    if (widget.aplicancestatus != null) {
      _appliancestatus.add(widget.aplicancestatus!);
      setState(() {
        _applianceData = widget.aplicancestatus;
      });
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
              // print('tttt;$_applianceData');
              // print('stream$snapshot');
              if (_applianceData != null) {
                return TabBarView(
                  children: _applianceData!.entries
                      .map((entry) => renderCount(entry.key, entry.value))
                      .toList(),
                  physics: NeverScrollableScrollPhysics(),
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
                        .toList(),
                    physics: NeverScrollableScrollPhysics());
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
    String roomname = key;
    return GridView.count(
      crossAxisCount: 2,
      childAspectRatio: 1.1,
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
                            air: widget.air ?? 0,
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
                    SizedBox(
                      height: 20,
                      child: Padding(
                        padding: EdgeInsets.only(top: 10.0),
                        child: Row(
                          mainAxisAlignment: MainAxisAlignment.end,
                          children: [
                            SwitchStream(
                              air: widget.air ?? 0,
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
                      ),
                    ),
                    Container(
                        decoration: BoxDecoration(
                            color: Colors.white,
                            borderRadius: BorderRadius.circular(100)),
                        child: SizedBox(
                            height: 100,
                            width: 100,
                            child: Image.asset(
                              'asset/img/$img.png',
                            ))),
                    Padding(
                      padding: EdgeInsets.symmetric(horizontal: 8.0),
                      child: Row(
                        children: [
                          Text(
                            title_name['${title}'] ?? '',
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
  final int air;
  const SwitchStream(
      {required this.onoff,
      required this.air,
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
                  air: air ?? 0,
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
            'id': 4,
            'to': 708002,
            'message': {
              'room': room,
              'device': title,
              'status': onoff == 'ON' ? 'OFF' : 'ON',
            }
          };
          String test = jsonEncode(A);
          SendMessage('appliance_status', test);
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
const Map<String, String> title_name = {
  'air_conditioner': '에어컨',
  'light': '전등',
  'tv': 'tv',
  'air_cleaner': '공기청정기'
};
