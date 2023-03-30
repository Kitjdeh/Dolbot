import 'dart:convert';

import 'package:http/http.dart' as http;

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
    // print(json["scheduleTime"].runtimeType);
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
  Map<String, String> headers = {
    'Content-Type': 'application/json',
    'Accept': 'application/json',
  };
  String url = 'http://j8a708.p.ssafy.io:8080/api/v1/schedule-info';
  http.Response response = await http.post(Uri.parse(url),
      body: json.encode(data), headers: headers);
  final int statusCode = response.statusCode;
  print(statusCode);
}
