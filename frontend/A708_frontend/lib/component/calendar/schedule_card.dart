import 'package:flutter/material.dart';
import 'package:intl/intl.dart';

class ScheduleCard extends StatelessWidget {
  final DateTime startTime;
  final String content;
  const ScheduleCard({required this.startTime, required this.content, Key? key})
      : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
        decoration: BoxDecoration(
          border: Border.all(
            width: 1.0,
            color: Colors.blue,
          ),
          borderRadius: BorderRadius.circular(8.0),
        ),
        child: Padding(
          padding: const EdgeInsets.all(16.0),
          child: IntrinsicHeight(
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                _Time(startTime: startTime),
                SizedBox(
                  width: 16.0,
                ),
                _Content(content: content),
                SizedBox(
                  width: 16.0,
                ),
              ],
            ),
          ),
        ));
  }
}

class _Time extends StatelessWidget {
  final DateTime startTime;

  const _Time({required this.startTime, Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final textStyle = TextStyle(
      fontWeight: FontWeight.w600,
      color: Colors.blue,
      fontSize: 16.0,
    );
    final formattedTime = DateFormat("HH:mm").format(startTime);
    // print('${formattedTime}formattedTime');
    // print(startTime);
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          '${formattedTime}',
          style: textStyle.copyWith(fontSize: 30),
        ),
        // Text(
        //   '${endTime.toString().padLeft(2, '0')}:00',
        //   style: textStyle.copyWith(fontSize: 10.0),
        // ),
      ],
    );
  }
}

class _Content extends StatelessWidget {
  final String content;

  const _Content({required this.content, Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Expanded(child: Text(content));
  }
}

class _Category extends StatelessWidget {
  final Color color;
  const _Category({required this.color, Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        color: Colors.blue[100],
        shape: BoxShape.circle,
      ),
      width: 16.0,
      height: 16.0,
    );
  }
}
