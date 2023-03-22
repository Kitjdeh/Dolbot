import 'package:dolbot/component/calendar/calendar.dart';
import 'package:dolbot/component/calendar/schedule_bottom_sheet.dart';
import 'package:dolbot/component/calendar/schedule_card.dart';
import 'package:dolbot/component/calendar/today_banner.dart';
import 'package:dolbot/component/home/weather_screen.dart';
import 'package:dolbot/database/drift_database.dart';
import 'package:dolbot/model/schedule_with_color.dart';

import 'package:flutter/material.dart';
import 'package:get_it/get_it.dart';

class CalendarScreen extends StatefulWidget {
  const CalendarScreen({Key? key}) : super(key: key);

  @override
  State<CalendarScreen> createState() => _CalendarScreenState();
}

class _CalendarScreenState extends State<CalendarScreen> {
  DateTime selectedDay = DateTime(
    DateTime.now().year,
    DateTime.now().month,
    DateTime.now().day,
  );
  DateTime focusedDay = DateTime.now();
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      floatingActionButton: renderFloatingActionButton(),
      body: Column(children: [
        Calendar(
          onDaySelected: onDaySelected,
          selectedDay: selectedDay,
          focusedDay: focusedDay,
        ),
        Padding(
          padding: const EdgeInsets.only(bottom: 8.0),
          child: ToadyBanner(selectedDay: selectedDay),
        ),
        _ScheduleList(
          selectedDate: selectedDay,
        ),
      ]),
    );
  }

  FloatingActionButton renderFloatingActionButton() {
    return FloatingActionButton(
      onPressed: () {
        showModalBottomSheet(
            context: context,
            isScrollControlled: true,
            builder: (_) {
              return SchedulBottomSheet(
                selectedDate: selectedDay,
              );
            });
      },
      backgroundColor: Colors.blue,
      child: Icon(
        Icons.add,
      ),
    );
  }

  onDaySelected(DateTime selectedDay, DateTime focusedDay) {
    setState(() {
      this.selectedDay = selectedDay;
      this.focusedDay = selectedDay;
    });
  }
}

class _ScheduleList extends StatelessWidget {
  final DateTime selectedDate;

  const _ScheduleList({required this.selectedDate, Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Expanded(
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 8.0),
        child: StreamBuilder<List<ScheduleWithColor>>(
            stream: GetIt.I<LocalDatabase>().watchSchedules(selectedDate),
            builder: (context, snapshot) {
              print(snapshot.data);
              if (!snapshot.hasData) {
                return Center(child: CircularProgressIndicator());
              }
              if (snapshot.hasData && snapshot.data!.isEmpty) {
                return Center(
                  child: Text('스케쥴일 없습니다.'),
                );
              }
              return ListView.separated(
                itemCount: snapshot.data!.length,
                separatorBuilder: (context, index) {
                  return SizedBox(height: 8.0);
                },
                itemBuilder: (context, index) {
                  final schedulWithColor = snapshot.data![index];
                  return Dismissible(
                    key: ObjectKey(schedulWithColor.schedule.id),
                    direction: DismissDirection.endToStart,
                    onDismissed: (DismissDirection direction) {
                      GetIt.I<LocalDatabase>()
                          .removeSchedule(schedulWithColor.schedule.id);
                    },
                    child: GestureDetector(
                      onTap: () {
                        showModalBottomSheet(
                            context: context,
                            isScrollControlled: true,
                            builder: (_) {
                              return SchedulBottomSheet(
                                selectedDate: selectedDate,
                                scheduleId: schedulWithColor.schedule.id,
                              );
                            });
                      },
                      child: ScheduleCard(
                          startTime: schedulWithColor.schedule.startTime,
                          endTime: schedulWithColor.schedule.endTime,
                          color: Color(int.parse(
                              'FF${schedulWithColor.categoryColor.hexCode}',
                              radix: 16)),
                          content: schedulWithColor.schedule.content),
                    ),
                  );
                },
              );
            }),
      ),
    );
  }
}
