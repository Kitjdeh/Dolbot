import 'package:flutter/material.dart';

class RobotBox extends StatelessWidget {
  String nickname;
  int robotNumber;
  int mainhomeId;
  RobotBox(
      {required this.mainhomeId,
      required this.robotNumber,
      required this.nickname,
      Key? key})
      : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
          color: robotNumber == mainhomeId ? Colors.blue[100] : Colors.white),
      child: Padding(
        padding: const EdgeInsets.all(8.0),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceAround,
          children: [
            Text(
              '$nickname 님의 기기',
              style: TextStyle(fontSize: 20),
            ),
            ElevatedButton(onPressed: () {}, child: Text('알림onoff'))
          ],
        ),
      ),
    );
  }
}
