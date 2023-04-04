import 'package:dolbot/component/login/kakao_login.dart';
import 'package:dolbot/const/tabs.dart';
import 'package:dolbot/screen/cctv_screen.dart';
import 'package:dolbot/screen/home_screen.dart';
import 'package:dolbot/screen/splash_screen.dart';
import 'package:flutter/material.dart';
import 'package:kakao_flutter_sdk/kakao_flutter_sdk_user.dart';
import 'package:dolbot/component/login/kakao_login.dart';

class MainScreen extends StatefulWidget {
  String? username;
  int? kakaoID;
  String? profile;

  MainScreen({this.username, this.kakaoID, this.profile, Key? key})
      : super(key: key);

  @override
  State<MainScreen> createState() => _MainScreenState();
}

class _MainScreenState extends State<MainScreen> with TickerProviderStateMixin {
  late final TabController controller;

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
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.blue[0],
      appBar: AppBar(
        backgroundColor: Colors.blue[100],
        title: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Row(
              children: [
                SizedBox( height:30, child: Image.network(widget?.profile ?? '')),
                Text(
                  '반가워요 ${widget?.username}님',
                  style: TextStyle(fontSize: 20, color: Colors.black),
                ),
              ],
            ),
            Text('${widget?.username} 홈',
                style: TextStyle(fontSize: 12, color: Colors.black))
          ],
        ),
      ),
      body: TabBarView(
        physics: NeverScrollableScrollPhysics(),
        controller: controller,
        children: WIDGETS.map((e) {
          return e;
        }).toList(),
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
