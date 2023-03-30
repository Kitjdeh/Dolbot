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

class _MainApplianceState extends State<MainAppliance>
    with TickerProviderStateMixin {
  late final TabController controller;
  IO.Socket? socket;

  Map? Appliance;
  final _appliancestatus = StreamController<Map>();
  String _selectindex = "0";
  @override
  // void dispose() {
  //   super.dispose();
  //   _cctvimage.close();
  // }
  // @override
  // void initState() {
  //   super.initState();
  //   // Connect to the socket server
  //   IO.Socket socket = IO.io('http://3.36.67.119:8081',
  //       IO.OptionBuilder().setTransports(['websocket']).build());
  //   socket.onConnect((_) {
  //     print('apllicane connect');
  //     // socket.emit('user_message', message);
  //   });
  //   // Listen for weather updates
  //   socket.on('appliance_status', (data) {
  //     print('33333 ${data.runtimeType}');
  //     var ImageData = jsonDecode(data);
  //     setState(() {
  //       RosImage = Image.memory(ImageData);
  //       _cctvimage.add(RosImage!);
  //     });
  //   });
  // }
  @override
  Widget build(BuildContext context) {
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
        body: StreamBuilder<Map>(
          stream: null,
          builder: (context, snapshot) {
            return TabBarView(
              // physics:,
              children: ROOM.map((e) => renderCount()).toList(),
            );
          }
        ),
      ),
    );
  }

  Widget renderCount() {
    return GridView.count(
      crossAxisCount: 2,
      children: ROOMINFO
          .map((e) => renderContainer(
              title: e.title, img: e.img, onoff: e.ONOFF, index: e.index))
          .toList(),
    );
  }

  Widget renderContainer({
    required String title,
    required String img,
    required bool onoff,
    required int index,
    int? temperature,

    // double? height,f
  }) {
    return Padding(
        padding: const EdgeInsets.all(8.0),
        child: GestureDetector(
          onTap: () {
            setState(() {
              _selectindex = title;
              if (onoff == true) {
                if (_selectindex == '에어컨') {
                  showModalBottomSheet(
                      context: context,
                      builder: (_) {
                        return AirCondition();
                      });
                } else if (_selectindex == '공기청정기') {
                  {
                    showModalBottomSheet(
                        context: context,
                        builder: (_) {
                          return AirCleaner();
                        });
                  }
                }
              }
            });
          },
          child: Container(
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(16),
                color: onoff == true ? Colors.blue[100] : Colors.grey[300],
              ),
              // height: height == null ? 300 : height,

              child: Center(
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    Row(
                      mainAxisAlignment: MainAxisAlignment.end,
                      children: [SwitchStream(onoff: 'off')],
                    ),
                    Container(
                        decoration: BoxDecoration(
                            color: Colors.white,
                            borderRadius: BorderRadius.circular(100)),
                        child: SizedBox(
                            height: 110,
                            width: 110,
                            child: Image.asset('$img'))),
                    Padding(
                      padding: const EdgeInsets.symmetric(horizontal: 8.0),
                      child: Row(
                        children: [
                          Text(
                            title,
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

  Widget selectCount() {
    return AirCondition();
  }

  Future<bool> getNumber(onoff) async {
    await Future.delayed(Duration(seconds: 3));
    final bool result = onoff == false ? true : false;
    print('요청');
    // throw Exception('에러 발생');
    return result;
  }
}

class RoomInfo {
  final String title;
  final bool ONOFF;
  final int? temperature;
  final int index;
  final String img;
  const RoomInfo(
      {required this.title,
      required this.ONOFF,
      required this.index,
      required this.img,
      this.temperature});
}

const ROOMINFO = [
  RoomInfo(
      title: '에어컨',
      img: 'asset/img/airconditional_on.png',
      ONOFF: true,
      temperature: 30,
      index: 0),
  RoomInfo(
      title: '공기청정기',
      img: 'asset/img/airfilter_off.png',
      ONOFF: false,
      index: 1),
  RoomInfo(title: '조명', img: 'asset/img/lamp_on.png', ONOFF: true, index: 2),
  RoomInfo(title: 'TV', img: 'asset/img/tv_on.png', ONOFF: true, index: 3),
];

class SwitchStream extends StatelessWidget {
  final String onoff;
  const SwitchStream({required this.onoff, Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
        child: Switch(
      value: onoff == 'on' ? true : false,
      onChanged: (value) {
        Map<String, dynamic> A = {
          'type': 'user',
          'id': 1,
          'to': 1,
          'message': {'room': 'living_room', 'device': 'tv', 'status': 'ON'}
        };
        String test = jsonEncode(A);
        SendMessage(test);

        // setState(() {
        //   onoff = value;
        // });
      },
      activeTrackColor: Colors.blue,
    ));
  }
}
