import 'package:dolbot/component/log/log_box.dart';
import 'package:dolbot/const/data.dart';
import 'package:dolbot/restapi/log_rest_api.dart';
import 'package:flutter/material.dart';
import 'package:intl/intl.dart';

class LogScreen extends StatefulWidget {
  // late final Future<Log>? LogData;

  LogScreen({Key? key}) : super(key: key);

  @override
  State<LogScreen> createState() => _LogScreenState();
}

class _LogScreenState extends State<LogScreen> {
  String _selectedDate = DateFormat('yyyy-MM-dd').format(DateTime.now());
  @override
  Widget build(BuildContext context) {
    late final Future<Log>? LogData = getLog(_selectedDate);

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
        final String? pictureUrl;
        pictureUrl = logdata?.pictureUrl;
        logs = logdata?.logs;
        return SingleChildScrollView(
          child: Column(
            children: [
              ElevatedButton(
                  onPressed: () {
                    _selectDate(context);
                  },
                  child: Text('$_selectedDate')),
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

  Future _selectDate(BuildContext context) async {
    final DateTime? selected = await showDatePicker(
        context: context,
        initialDate: DateTime.now(),
        firstDate: DateTime(2010),
        lastDate: DateTime(2030));
    if (selected != null) {
      setState(() {
        _selectedDate = DateFormat('yyyy-MM-dd').format(selected);
        print(_selectedDate);
      });
    }
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
