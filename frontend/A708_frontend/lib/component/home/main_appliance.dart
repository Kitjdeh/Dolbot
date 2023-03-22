import 'dart:convert';

import 'package:dolbot/component/home/air_cleaner.dart';
import 'package:dolbot/component/home/aircondition.dart';
import 'package:dolbot/component/home/lamp_tv.dart';
import 'package:dolbot/const/data.dart';
import 'package:dolbot/sockect/sockect.dart';
import 'package:flutter/material.dart';

class MainAppliance extends StatefulWidget {
  const MainAppliance({Key? key}) : super(key: key);

  @override
  State<MainAppliance> createState() => _MainApplianceState();
}

class _MainApplianceState extends State<MainAppliance>
    with TickerProviderStateMixin {
  late final TabController controller;
  String _selectindex = "0";

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
                    unselectedLabelStyle:
                        TextStyle(fontWeight: FontWeight.w100),
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
          body: TabBarView(
            // physics:,
            children: APPLIANCE.map((e) => renderCount()).toList(),
          ),
        ));
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
              showModalBottomSheet(
                  context: context,
                  builder: (_) {
                    if (_selectindex == '에어컨') {
                      return AirCondition();
                    } else if (_selectindex == '공기청정기') {
                      return AirCleaner();
                    } else {
                      return LampTv();
                    }
                  });
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
                      children: [
                        Switch(
                          value: onoff,
                          onChanged: (value) {
                            Map<String, dynamic> A = {
                              'type': 'user',
                              'id': 1,
                              'to': 1,
                              'message': {
                                'room': 1,
                                'device': 'tv',
                                'status': 'on'
                              }
                            };
                            String test = jsonEncode(A);
                            connectAndListen(test);

                            setState(() {
                              onoff = value;
                            });
                          },
                          activeTrackColor: Colors.blue,
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

  Widget renderCount() {
    return GridView.count(
      crossAxisCount: 2,
      children: ROOMINFO
          .map((e) => renderContainer(
              title: e.title, img: e.img, onoff: e.ONOFF, index: e.index))
          .toList(),
    );
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
