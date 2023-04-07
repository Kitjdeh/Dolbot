import 'dart:convert';

import 'package:dolbot/component/alert/toast.dart';
import 'package:dolbot/sockect/sockect.dart';
import 'package:flutter/material.dart';
import 'package:sleek_circular_slider/sleek_circular_slider.dart';
import 'dart:io';

Map<String, int> power = {'low': 0, 'middle': 1, 'high': 2};
Map<String, int> Mode = {'low': 0, 'middle': 1, 'high': 2};
List<bool> isSelectedMode = [false, false, false];
List<bool> isSelectedPower = [false, false, false];

class AirCondition extends StatefulWidget {
  AirCondition(
      {required this.speed,
      required this.target,
      required this.mode,
      required this.room,
      Key? key})
      : super(key: key);
  String mode;
  String speed;
  String room;
  int target;
  @override
  State<AirCondition> createState() => _AirConditionState();
}

class _AirConditionState extends State<AirCondition> {
  @override
  Widget build(BuildContext context) {
    int power_num = power[widget.speed] ?? 0;
    int mode_num = Mode[widget.mode] ?? 0;
    // isSelectedMode[mode_num] = true;
    // isSelectedPower[power_num] = true;

    // for (int idx = 0; idx < isSelectedMode.length; idx++) {
    //   if (idx == power_num) {
    //     isSelectedPower[idx] = true;
    //   } else {
    //     isSelectedPower[idx] = false;
    //   }
    //   if (idx == mode_num) {
    //     isSelectedMode[idx] = true;
    //   } else {
    //     isSelectedMode[idx] = false;
    //   }
    // }
    print('1111$isSelectedMode , ${mode_num} ,${widget.mode}');
    return Container(
        child: Column(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            crossAxisAlignment: CrossAxisAlignment.center,
            children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Text(
                '에어컨',
                style: TextStyle(fontSize: 30, fontWeight: FontWeight.w700),
              ),
            ],
          ),
          Container(
            decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(100),
                image: new DecorationImage(
                    image: widget.mode == 'cool'
                        ? new AssetImage('asset/img/weather_snow.gif')
                        : widget.mode == 'warm'
                            ? new AssetImage('asset/img/weather_hot.gif')
                            : new AssetImage('asset/img/weather_fog.gif'),
                    fit: BoxFit.cover)),
            child: SleekCircularSlider(
              appearance: CircularSliderAppearance(
                  customWidths: CustomSliderWidths(),
                  size: MediaQuery.of(context).size.height / 10 * 3,
                  infoProperties: InfoProperties(
                      bottomLabelText: '희망온도',
                      bottomLabelStyle: TextStyle(
                        fontSize: 20,
                        fontWeight: FontWeight.w700,
                        color: Colors.white,
                      ),
                      mainLabelStyle: TextStyle(
                        fontSize: 50.0,
                        fontWeight: FontWeight.w700,
                        color: Colors.white,
                      ),
                      modifier: percentageModifier),
                  customColors: CustomSliderColors(
                      trackColor: Colors.blue[100],
                      progressBarColor: Colors.blue)),
              initialValue: widget.target.toDouble(),
              onChange: (double value) {
                setState(() {
                  widget.target = value.toInt();
                });
              },
              max: 50,
              min: 0,
            ),
          ),
          SizedBox(
            height: 30,
            child: Row(
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
                      for (int buttonIndex = 0;
                          buttonIndex < isSelectedMode.length;
                          buttonIndex++) {
                        if (buttonIndex == index) {
                          isSelectedMode[buttonIndex] = true;
                          if (buttonIndex == 0) {
                            widget.mode = 'cool';
                          } else if (buttonIndex == 1) {
                            widget.mode = 'warm';
                          } else if (buttonIndex == 2) {
                            widget.mode = 'dry';
                          }
                        } else {
                          isSelectedMode[buttonIndex] = false;
                        }
                      }
                      print(isSelectedMode);
                      // _isSelectedMode = isSelectedMode;
                      mode_num = index;
                    });
                  },
                  children: [Text('냉방'), Text('난방'), Text('제습')],
                ),
              ],
            ),
          ),
          SizedBox(
            height: 30,
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                Text(
                  '바람세기',
                  style: TextStyle(fontWeight: FontWeight.w700),
                ),
                ToggleButtons(
                  textStyle: TextStyle(fontWeight: FontWeight.w700),
                  selectedBorderColor: Colors.blue,
                  borderRadius: BorderRadius.circular(100),
                  isSelected: isSelectedPower,
                  onPressed: (int index) {
                    setState(() {
                      for (int buttonIndex = 0;
                          buttonIndex < isSelectedPower.length;
                          buttonIndex++) {
                        if (buttonIndex == index) {
                          isSelectedPower[buttonIndex] = true;
                          if (buttonIndex == 0) {
                            widget.speed = 'low';
                          } else if (buttonIndex == 1) {
                            widget.speed = 'mid';
                          } else if (buttonIndex == 2) {
                            widget.speed = 'high';
                          }
                        } else {
                          isSelectedPower[buttonIndex] = false;
                        }
                      }
                    });
                  },
                  children: [Text('약풍'), Text('중풍'), Text('강풍')],
                ),
              ],
            ),
          ),
          Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              ElevatedButton(
                  onPressed: () {
                    Map<String, dynamic> A = {
                      'type': 'user',
                      'id': 4,
                      'to': 708002,
                      'message': {
                        'room': widget.room,
                        'device': 'air_conditioner',
                        'status': 'ON',
                        "mode": widget.mode,
                        "speed": widget.speed,
                        "target": widget.target
                      }
                    };
                    String message = jsonEncode(A);
                    SendMessage('appliance_status', message);
                    toast(context, '기기 명령이 전달되었습니다.');
                    print(message);
                  },
                  child: Text('작동')),
            ],
          )
        ]));
  }
}

String percentageModifier(double value) {
  final roundedValue = value.ceil().toInt().toString();
  return '$roundedValue ℃';
}

void onSliderChanged(double val) {}
