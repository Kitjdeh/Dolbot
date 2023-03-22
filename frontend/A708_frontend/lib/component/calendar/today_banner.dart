import 'package:dolbot/database/drift_database.dart';
import 'package:dolbot/model/schedule_with_color.dart';
import 'package:flutter/material.dart';
import 'package:get_it/get_it.dart';
class ToadyBanner extends StatelessWidget {
  final DateTime selectedDay;

  const ToadyBanner(
      {required this.selectedDay,  Key? key})
      : super(key: key);

  @override
  Widget build(BuildContext context) {
    final textStyle = TextStyle(
      fontWeight: FontWeight.w600,
      color: Colors.black,
    );
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
            StreamBuilder<List<ScheduleWithColor>>(
                stream: GetIt.I<LocalDatabase>().watchSchedules(selectedDay),
                builder: (context, snapshot) {
                  int count = 0;
                  if (snapshot.hasData){
                    count=  snapshot.data!.length;
                  }
                  return Text(
                    '${count}개',
                    style: textStyle,
                  );
                }
            )
          ],
        ),
      ),
    );
  }
}
