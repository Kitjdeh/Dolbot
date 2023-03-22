import 'package:flutter/material.dart';

Widget LogBox({
  required int index,
  required String location,
  required String content,
  required int endtime,
}) {
  return Column(
    children: [
      Container(
        width: 350,
        height: 80,
        decoration: BoxDecoration(
          color: index == 1
              ? Colors.yellow[100]
              : index == 0
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
                      color: index == 1
                          ? Colors.red[400]
                          : index == 0
                              ? Colors.blue[400]
                              : Colors.green[400],
                      borderRadius: BorderRadius.circular(16)),
                  child: SizedBox(
                    width: 40,
                    height: 35,
                    child: Row(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Text(location,
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
                    child: Text(
                  content,
                  style: TextStyle(fontSize: 15),
                )),
              ),
              Column(
                mainAxisAlignment: MainAxisAlignment.start,
                children: [
                  Text(
                    endtime.toString(),
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
