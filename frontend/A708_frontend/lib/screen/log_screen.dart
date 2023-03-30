import 'package:dolbot/component/log/log_box.dart';
import 'package:dolbot/const/data.dart';
import 'package:dolbot/restapi/log_rest_api.dart';
import 'package:flutter/material.dart';

class LogScreen extends StatelessWidget {
  // late final Future<Log>? LogData;

  LogScreen({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    late final Future<Log>? LogData = getLog();

    return FutureBuilder<Log>(
      future: LogData,
      builder: (context, snapshot) {
        if (snapshot.connectionState == ConnectionState.waiting) {
          return SizedBox(height: 150, child: CircularProgressIndicator());
        }

        if (snapshot.hasError) {
          return Text('Error: ${snapshot.error}');
        }

        final logdata = snapshot.data;
        final List<dynamic>? logs;
        final String?  pictureUrl;
        pictureUrl = logdata?.pictureUrl;
        logs = logdata?.logs;
        return SingleChildScrollView(
          child: Column(
            children: [
              Image.network('${pictureUrl}'),
              Column(
                children: [
                  renderSimple(logs),
                ],
              ),
            ],
          ),
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
