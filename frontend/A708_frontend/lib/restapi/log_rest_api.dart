import 'dart:convert';

import 'package:http/http.dart' as http;

Future<Log> getLog() async {
  final response = await http.get(Uri.parse(
      'http://j8a708.p.ssafy.io:8080/api/v1/log/logs/1?localDate=2023-03-30'));

  if (response.statusCode == 200) {

    // Log dataList = jsonDecode(utf8.decode(response.bodyBytes));
    // print(dataList);
    // var Logs = dataList;
    final decodedBody = utf8.decode(response.bodyBytes);
    return Log.fromJson(jsonDecode(decodedBody));

  } else {
    throw Exception('Failed to load 로오오옹오오오오오그');
  }
}

class Log {
  final int? robotId;
  final int? logListId;
  final List<int>? logDate;
  final String? pictureUrl;
  final String? startDate;
  final List<Map<String, dynamic>>? logs;
  Log(
      {this.robotId,
      this.logListId,
      this.logDate,
      this.pictureUrl,
      this.startDate,
      this.logs});

  factory Log.fromJson(Map<String, dynamic> json) {
    var logFromJson = json["logDate"];
    List<int> logList = new List<int>.from(logFromJson);

    var logsFromJson = json["logs"];
    List<Map<String, dynamic>> logsList = List<Map<String, dynamic>>.from(
        logsFromJson.map((x) => Map<String, dynamic>.from(x)));

    return Log(
      robotId: json["robotId"],
      logListId: json["logListId"],
      logDate: logList,
      pictureUrl: json["pictureUrl"],
      startDate: json["startDate"],
      logs: logsList,
    );
  }
}
// factory Log.fromJson(Map<String, dynamic> json) {
// // print(json["scheduleTime"].runtimeType);
// // print(json["content"].runtimeType);
// // print(json["startDate"].runtimeType);
// var logFromJson = json["logDate"];
// List<int> logList = new List<int>.from(logFromJson);
// return Log(
// robotId: json["robotId"],
// logListId: json["logListId"],
// logDate: logList,
// pictureUrl: json["pictureUrl"],
// startDate: json["startDate"],
// logs: json["logs"]);
// }
