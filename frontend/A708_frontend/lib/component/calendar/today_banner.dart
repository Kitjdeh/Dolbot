import 'package:dolbot/restapi/calendar_rest_api.dart';
import 'package:flutter/material.dart';
import 'package:get_it/get_it.dart';
import 'package:intl/intl.dart';
import 'dart:async';

class ToadyBanner extends StatelessWidget {
  final DateTime selectedDay;
  late Future<List<Schedule>>? ScheduleList;
  ToadyBanner({required this.selectedDay, Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    final textStyle = TextStyle(
      fontWeight: FontWeight.w600,
      color: Colors.black,
    );

    String? _selectedDate = DateFormat('yyyy-MM-dd').format(selectedDay);

    ScheduleList = getSchedule(_selectedDate);
    return Container(
      color: Colors.blue,
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 16.0, vertical: 8.0),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Text(
              '${selectedDay.year}년 ${selectedDay.month}월 ${selectedDay.day}일',
              style: textStyle,
            ),
            FutureBuilder<List<Schedule>>(
                future: ScheduleList,
                builder: (context, snapshot) {
                  int count = 0;
                  if (snapshot.hasData) {
                    count = snapshot.data!.length;
                  }

                  return Text(
                    '${count}개',
                    style: textStyle,
                  );
                })
          ],
        ),
      ),
    );
  }
}
