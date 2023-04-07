import 'dart:convert';

import 'package:dolbot/sockect/sockect.dart';
import 'package:http/http.dart' as http;
import 'package:intl/intl.dart';

Future<List<Schedule>> getSchedule(date) async {
  final response = await http.get(Uri.parse(
      'http://j8a708.p.ssafy.io:8080/api/v1/schedule-info/1?localDate=${date}'));

  if (response.statusCode == 200) {
    List dataList = jsonDecode(utf8.decode(response.bodyBytes));

    var Schedules = dataList.map((e) => Schedule.fromJson(e)).toList();
    return Schedules;
  } else {
    throw Exception('Failed to load album');
  }
}

class Schedule {
  final int? scheduleId;
  final int? homeId;
  final int? scheduleTime;
  final String? content;
  final String? startDate;
  final String? endDate;

  Schedule(
      {this.scheduleId,
      this.homeId,
      this.scheduleTime,
      this.content,
      this.endDate,
      this.startDate});

  factory Schedule.fromJson(Map<String, dynamic> json) {
    // DateTime datetime =
    //     DateTime.fromMillisecondsSinceEpoch(json["scheduleTime"], isUtc: true)
    // .add(Duration(hours: 9));
    // print('시간 해독 ${datetime}');
    // print('스케쥴타임 ${DateFormat('HH:mm').format(json["scheduleTime"])}');
    // print(json["content"].runtimeType);
    // print(json["startDate"].runtimeType);
    return Schedule(
        scheduleId: json["scheduleId"],
        homeId: json["homeId"],
        scheduleTime: json["scheduleTime"],
        content: json["content"],
        startDate: json["startDate"],
        endDate: json["endDate"]);
  }
}

Future<dynamic> postSchedule(dynamic data) async {
  // int Id = data['userid'];
  // print(Id);
  Map<String, dynamic> msg = {
    'type': 'user',
    'id': 4,
    'to': 708002,
    'message': ''
  };
  String message = jsonEncode(msg);
  SendMessage('schedule', message);
  Map<String, String> headers = {
    'Content-Type': 'application/json',
    'Accept': 'application/json',
  };
  String url = 'http://j8a708.p.ssafy.io:8080/api/v1/schedule-info';
  http.Response response = await http.post(Uri.parse(url),
      body: json.encode(data), headers: headers);
  final int statusCode = response.statusCode;
}

Future<dynamic> patchSchedule(dynamic data) async {
  Map<String, dynamic> msg = {
    'type': 'user',
    'id': 4,
    'to': 708002,
    'message': ''
  };
  String message = jsonEncode(msg);
  SendMessage('schedule', message);
  Map<String, String> headers = {
    'Content-Type': 'application/json',
    'Accept': 'application/json',
  };
  String url = 'http://j8a708.p.ssafy.io:8080/api/v1/schedule-info';
  http.Response response = await http.patch(Uri.parse(url),
      body: json.encode(data), headers: headers);
  final int statusCode = response.statusCode;
}
