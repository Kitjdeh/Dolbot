import 'package:dolbot/component/log/log_box.dart';
import 'package:dolbot/const/data.dart';
import 'package:dolbot/restapi/log_rest_api.dart';
import 'package:flutter/material.dart';
import 'package:intl/intl.dart';

class DailyLog extends StatefulWidget {
  // late final Future<Log>? LogData;
  String selectedDate;
  DailyLog({required this.selectedDate, Key? key}) : super(key: key);

  @override
  State<DailyLog> createState() => _DailyLogState();
}

class _DailyLogState extends State<DailyLog> {
  @override
  Widget build(BuildContext context) {
    late final Future<Log>? LogData = getLog(widget.selectedDate);

    return FutureBuilder<Log>(
      future: LogData,
      builder: (context, snapshot) {
        if (snapshot.connectionState == ConnectionState.waiting) {
          return SizedBox(height: 100, child: CircularProgressIndicator());
        }
        if (snapshot.hasError) {
          return Text('Error: ${snapshot.error}');
        }
        final logdata = snapshot.data;
        final List<dynamic>? logs;
        final String? pictureUrl;
        pictureUrl = logdata?.pictureUrl;
        logs = logdata?.logs;
        return Column(
          children: [
            Image.network('${pictureUrl}'),
            Padding(
              padding: EdgeInsets.all(16.0),
              child: Text(
                '${widget.selectedDate.toString()}의 기록',
                style: TextStyle( fontSize: 15),
              ),
            ),
            Column(
              children: [
                renderSimple(logs),
              ],
            ),
          ],
        );
      },
    );
  }

  Widget renderSimple(List<dynamic>? log) {
    return Container(
      decoration: BoxDecoration(color: Colors.white),
      child: Column(
        children: [
          Column(
            children: log
                    ?.map(
                      (e) => LogBox(
                          type: (e["type"]),
                          location: e["roomName"],
                          applianceName: e["applianceName"],
                          content: e["scheduleContent"],
                          endtime: (e["logTime"]),
                          onoff: e["on"]),
                    )
                    ?.toList() ??
                [],
          )
        ],
      ),
    );
  }
}
