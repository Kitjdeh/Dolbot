import 'dart:convert';

import 'package:dolbot/sockect/sockect.dart';
import 'package:flutter/material.dart';
import 'package:path/path.dart';

class CctvManualView extends StatelessWidget {
  int user;
  int robotnumber;
  CctvManualView({required this.user, required this.robotnumber, Key? key})
      : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(color: Colors.blue[200]),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        crossAxisAlignment: CrossAxisAlignment.center,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              RoomButton(
                roomid: 'living_room',
                roomname: '거실',
                user: user,
                robotnumber: robotnumber,
              ),
              RoomButton(
                  roomid: 'inner_room',
                  roomname: '안방',
                  user: user,
                  robotnumber: robotnumber),
              RoomButton(
                  roomid: 'library',
                  roomname: '서재',
                  user: user,
                  robotnumber: robotnumber)
            ],
          ),
          SizedBox(
            height: 10,
          ),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              RoomButton(
                  roomid: 'small_room',
                  roomname: '작은방',
                  user: user,
                  robotnumber: robotnumber),
              RoomButton(
                  roomid: 'entrance',
                  roomname: '현관',
                  user: user,
                  robotnumber: robotnumber),
              RoomButton(
                  roomid: 'kitchen',
                  roomname: '주방',
                  user: user,
                  robotnumber: robotnumber)
            ],
          ),
        ],
      ),
    );
  }
}

class RoomButton extends StatelessWidget {
  final String roomid;
  final String roomname;
  final int user;
  final int robotnumber;
  const RoomButton({
    required this.user,
    required this.robotnumber,
    required this.roomname,
    required this.roomid,
  });

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: 100,
      child: ElevatedButton(
          onPressed: () async {
            Map<String, dynamic> data = {
              'type': 'user',
              'id': user,
              'to': 708002,
              'message': '',
              'room': roomid,
            };
            String message = jsonEncode(data);
            print(message);
            SendMessage('cctv', message);
          },
          child: Text(
            '$roomname',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.w700),
          )),
    );
  }
}
List<bool> selectedroom = [false, false, false, false];
List<String> roomName = ['안방', '거실', '화장실', '방1'];
int? selectednumber;
