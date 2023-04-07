import 'package:flutter/material.dart';

Widget LogBox({
  required String type,
  required String location,
  required String applianceName,
  required String content,
  required String endtime,
  required bool onoff,
}) {
  return Column(
    children: [
      Container(
        width: 350,
        height: 80,
        decoration: BoxDecoration(
          color: type == '일정'
              ? Colors.yellow[100]
              : type == '가전'
                  ? Colors.blue[50]
                  : Colors.grey[100],
        ),
        child: Padding(
          padding: const EdgeInsets.all(16.0),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Padding(
                padding: EdgeInsets.only(right: 16.0),
                child: Container(
                  width: 70,
                  decoration: BoxDecoration(
                      color: type == '비상'
                          ? Colors.red[400]
                          : type == '가전'
                              ? Colors.blue[400]
                              : Colors.green[400],
                      borderRadius: BorderRadius.circular(16)),
                  child: SizedBox(
                    width: 40,
                    height: 35,
                    child: Row(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        type == '비상'
                            ? Text(location,
                                style: TextStyle(
                                    fontSize: 20,
                                    fontWeight: FontWeight.w700,
                                    color: Colors.white))
                            : type == '일정'
                                ? Text(location,
                                    style: TextStyle(
                                        fontSize: 20,
                                        fontWeight: FontWeight.w700,
                                        color: Colors.white))
                                : Text('가전',
                                    style: TextStyle(
                                        fontSize: 20,
                                        fontWeight: FontWeight.w700,
                                        color: Colors.white)),
                      ],
                    ),
                  ),
                ),
              ),
              Expanded(
                child: Container(
                  child: type == '가전'
                      ? Text(
                          '${location} ${applianceName} ${onoff.toString() == true ? '켜졌습니다.' : '꺼졌습니다.'}',
                          style: TextStyle(fontSize: 15))
                      : type == '일정'
                          ? Text('${content}', style: TextStyle(fontSize: 15))
                          : Text('비상', style: TextStyle(fontSize: 15)),
                ),
              ),
              Column(
                mainAxisAlignment: MainAxisAlignment.start,
                children: [
                  Text(
                    endtime,
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
      SizedBox(
        height: 16,
      )
    ],
  );
}

const rainbowColors = [
  Colors.green,
  Colors.blue,
  Colors.red,
  Colors.orange,
  Colors.yellow,
  Colors.indigo,
  Colors.purple
];
Map<String, dynamic> APPLIANCE = {};
