import 'dart:convert';
import 'package:dolbot/component/log/log_box.dart';
import 'package:dolbot/component/login/kakao_login.dart';
import 'package:dolbot/component/menu/robot_box.dart';
import 'package:dolbot/restapi/user_rest_api.dart';
import 'package:dolbot/screen/register_screen.dart';
import 'package:dolbot/screen/splash_screen.dart';
import 'package:dolbot/sockect/sockect.dart';
import 'package:http/http.dart' as http;
import 'package:flutter/material.dart';
import 'package:socket_io_client/socket_io_client.dart' as IO;
import 'dart:async';

class MenuScreen extends StatefulWidget {
  int userid;
  int mainhomeId;
  int kakaoId;
  String profile;
  String username;
  MenuScreen(
      {required this.username,
      required this.profile,
      required this.kakaoId,
      required this.mainhomeId,
      required this.userid,
      Key? key})
      : super(key: key);

  @override
  State<MenuScreen> createState() => _MenuScreenState();
}

class _MenuScreenState extends State<MenuScreen> {
  final viewModel = MainViewModel(KakaoLogin());
  late Future<List<Robot>>? RobotList;

  @override
  Widget build(BuildContext context) {
    RobotList = getRobots(widget.userid);
    return Padding(
      padding: const EdgeInsets.only(top: 32.0),
      child: Container(
        child: FutureBuilder<List<Robot>>(
            future: RobotList,
            builder: (context, snapshot) {
              if (!snapshot.hasData) {
                return Center(child: CircularProgressIndicator());
              }
              if (snapshot.hasData && snapshot.data!.isEmpty) {
                return Center(
                  child: Text('로봇이 없습니다.'),
                );
              }
              return Column(
                mainAxisAlignment: MainAxisAlignment.spaceAround,
                children: [
                  Column(
                    children: [
                      Row(
                        mainAxisAlignment: MainAxisAlignment.start,
                        children: [
                          Text('연결된 기기(${snapshot.data!.length})'),
                        ],
                      ),
                      Container(
                        height: 300,
                        child: ListView.separated(
                          itemCount: snapshot.data!.length,
                          separatorBuilder: (context, index) {
                            return SizedBox(
                              height: 8.0,
                            );
                          },
                          itemBuilder: (context, index) {
                            final robot = snapshot.data![index];
                            return RobotBox(
                                mainhomeId: widget.mainhomeId,
                                robotNumber: robot.homeId ?? -1,
                                nickname: robot.nickname ?? '');
                          },
                        ),
                      ),
                    ],
                  ),
                  Column(
                    children: [
                      ElevatedButton(
                          onPressed: () {
                            Navigator.of(context).pushReplacement(
                              MaterialPageRoute(
                                builder: (BuildContext context) =>
                                    RegisterScreen(
                                        userId: widget.userid,
                                        username: widget.username,
                                        kakaoID: widget.kakaoId,
                                        profile: widget.profile),
                              ),
                            );
                          },
                          child: Text('기기추가')),
                      ElevatedButton(
                        onPressed: () async {
                          await viewModel.logout();
                          await Navigator.of(context).pushReplacement(
                              MaterialPageRoute(
                                  builder: (BuildContext context) =>
                                      SplashScreen()));
                          setState(() {});
                        },
                        child: const Text('Logout'),
                      )
                    ],
                  ),
                ],
              );
            }),
      ),
    );
    // By default, show a loading spinner.
  }
}
