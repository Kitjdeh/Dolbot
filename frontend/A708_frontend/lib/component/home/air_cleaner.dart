import 'dart:convert';

import 'package:dolbot/component/alert/toast.dart';
import 'package:dolbot/sockect/sockect.dart';
import 'package:flutter/material.dart';

Map<String, int> numbering = {'low': 0, 'middle': 1, 'high': 2};

class AirCleaner extends StatefulWidget {
  String mode;
  String room;
  int air;

  AirCleaner(
      {required this.air, required this.mode, required this.room, Key? key})
      : super(key: key);

  @override
  State<AirCleaner> createState() => _AirCleanerState();
}

class _AirCleanerState extends State<AirCleaner> {
  List<bool> isSelectedMode = [false, false, false];

  @override
  Widget build(BuildContext context) {
    int num = numbering[widget.mode] ?? 0;
    isSelectedMode[num] = true;
    return Container(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        crossAxisAlignment: CrossAxisAlignment.center,
        children: [
          Text(
            '공기정청기',
            style: TextStyle(fontSize: 20, fontWeight: FontWeight.w700),
          ),
          Container(
            width: 250,
            height: 250,
            decoration: BoxDecoration(
                boxShadow: [
                  BoxShadow(
                    color: Colors.greenAccent[100]!,
                    blurRadius: 10.0,
                    spreadRadius: 5.0,
                  )
                ],
                border:
                    Border.all(color: Colors.greenAccent[100]!, width: 10.0),
                color: Colors.greenAccent,
                borderRadius: BorderRadius.circular(250.0)),
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Text('상태',
                    style:
                        TextStyle(fontSize: 15, fontWeight: FontWeight.w700)),
                Container(
                    width: MediaQuery.of(context).size.height / 5,
                    decoration:
                        BoxDecoration(borderRadius: BorderRadius.circular(100)),
                    child: widget.air > 80
                        ? Image.asset('asset/img/smile.gif')
                        : widget.air > 50
                            ? Image.asset('asset/img/normal.gif')
                            : Image.asset('asset/img/angry.gif')),
                widget.air > 80
                    ? Text(
                        '좋아요',
                        style: TextStyle(
                            fontSize: 20, fontWeight: FontWeight.w700),
                      )
                    : widget.air > 50
                        ? Text(
                            '보통',
                            style: TextStyle(
                                fontSize: 20, fontWeight: FontWeight.w700),
                          )
                        : Text(
                            '나빠요',
                            style: TextStyle(
                                fontSize: 20, fontWeight: FontWeight.w700),
                          ),
              ],
            ),
          ),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              Text(
                '모드선택',
                style: TextStyle(fontWeight: FontWeight.w700),
              ),
              ToggleButtons(
                textStyle: TextStyle(fontWeight: FontWeight.w700),
                selectedBorderColor: Colors.blue,
                borderRadius: BorderRadius.circular(100),
                isSelected: isSelectedMode,
                onPressed: (int index) {
                  setState(() {
                    print(index);
                    for (int buttonIndex = 0; buttonIndex < 3; buttonIndex++) {
                      if (buttonIndex == index) {
                        isSelectedMode[buttonIndex] = true;
                        if (buttonIndex == 0) {
                          widget.mode = 'low';
                        } else if (buttonIndex == 1) {
                          widget.mode = 'mid';
                        } else if (buttonIndex == 2) {
                          widget.mode = 'high';
                        }
                      } else {
                        isSelectedMode[buttonIndex] = false;
                      }
                    }
                  });
                },
                children: [Text('약풍'), Text('중풍'), Text('강풍')],
              ),
            ],
          ),
          ElevatedButton(
              onPressed: () {
                Map<String, dynamic> A = {
                  'type': 'user',
                  'id': 4,
                  'to': 708002,
                  'message': {
                    'room': widget.room,
                    'device': 'air_cleaner',
                    'status': 'ON',
                    "mode": widget.mode
                  }
                };
                String message = jsonEncode(A);
                SendMessage('appliance_status', message);
                toast(context, '기기 명령이 전달되었습니다.');
              },
              child: Text('작동'))
        ],
      ),
    );
  }
}
