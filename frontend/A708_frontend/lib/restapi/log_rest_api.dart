import 'dart:convert';

import 'package:http/http.dart' as http;

Future<Log> getLog(date) async {
  final response = await http.get(
      Uri.parse('https://j8a708.p.ssafy.io/api/v1/log/logs/2?localDate=$date'));
  if (response.statusCode == 200) {
    // Log dataList = jsonDecode(utf8.decode(response.bodyBytes));
    // print(dataList);
    // var Logs = dataList;
    final decodedBody = utf8.decode(response.bodyBytes);
    return Log.fromJson(jsonDecode(decodedBody));
  } else {
    throw Exception('해당일에는 로그가 없습니다.');
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
