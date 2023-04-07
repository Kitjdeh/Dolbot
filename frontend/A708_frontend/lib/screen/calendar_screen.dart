import 'dart:convert';
import 'package:dolbot/component/calendar/calendar.dart';
import 'package:dolbot/component/calendar/date_field.dart';
import 'package:dolbot/component/calendar/register_schedule.dart';
import 'package:dolbot/component/calendar/schedule_bottom_sheet.dart';
import 'package:dolbot/component/calendar/schedule_card.dart';
import 'package:dolbot/component/calendar/today_banner.dart';
import 'package:intl/date_symbol_data_local.dart';
import 'package:intl/intl.dart';
import 'package:flutter/material.dart';
import 'package:get_it/get_it.dart';
import 'package:http/http.dart' as http;
import 'dart:async';

import '../restapi/calendar_rest_api.dart';

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
  void didChangeDependencies() {
    super.didChangeDependencies();
    initializeDateFormatting(Localizations.localeOf(context).languageCode);
  }
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
                homeId: null,
                scheduleTime: null,
                content: null,
                startDate: DateFormat('yyyy-MM-dd').format(DateTime.now()),
                endDate: null,
                scheduleId: null,
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
  late Future<List<Schedule>>? ScheduleList;

  _ScheduleList({required this.selectedDate, Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    String? _selectedDate = DateFormat('yyyy-MM-dd').format(selectedDate);

    ScheduleList = getSchedule(_selectedDate);
    return Expanded(
      child: Padding(
        padding: EdgeInsets.symmetric(horizontal: 8.0),
        child: FutureBuilder<List<Schedule>>(
            future: ScheduleList,
            builder: (context, snapshot) {
              if (!snapshot.hasData) {
                return Center(child: CircularProgressIndicator());
              }
              if (snapshot.hasData && snapshot.data!.isEmpty) {
                return Center(
                  child: Text('스케쥴이 없습니다.'),
                );
              }
              return ListView.separated(
                itemCount: snapshot.data!.length,
                separatorBuilder: (context, index) {
                  return SizedBox(height: 8.0);
                },
                itemBuilder: (context, index) {
                  final todayschedule = snapshot.data![index];
                  DateTime datetime = DateTime.fromMillisecondsSinceEpoch(
                      todayschedule.scheduleTime! ,isUtc: true).add(Duration(hours: 9));
                  print('datetime ${datetime} ${todayschedule.scheduleId}/// ${todayschedule.scheduleTime!}');
                  return Dismissible(
                    key: ObjectKey(todayschedule.scheduleId),
                    direction: DismissDirection.endToStart,
                    // onDismissed: (DismissDirection direction) {
                    //   GetIt.I<LocalDatabase>()
                    //       .removeSchedule(schedulWithColor.schedule.id);
                    // },
                    child: GestureDetector(
                      onTap: () {
                        showModalBottomSheet(
                            context: context,
                            isScrollControlled: true,
                            builder: (_) {
                              return SchedulBottomSheet(
                                homeId: todayschedule.homeId ?? null,
                                scheduleTime: todayschedule.scheduleTime,
                                content: todayschedule.content,
                                startDate: todayschedule.startDate,
                                endDate: todayschedule.endDate,
                                selectedDate: selectedDate,
                                scheduleId: todayschedule.scheduleId,
                              );
                            });
                      },
                      child: ScheduleCard(
                          startTime: datetime,
                          // endTime: DateTime.parse('${todayschedule.endDate}'),
                          content: todayschedule.content!.toString()),
                    ),
                  );
                },
              );
            }),
      ),
    );
  }
}
