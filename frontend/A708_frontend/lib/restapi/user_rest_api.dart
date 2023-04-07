import 'dart:convert';
import 'package:http/http.dart' as http;

Future postLogin(kakaoId) async {
  Map<String, String> headers = {
    'Content-Type': 'application/json',
    'Accept': 'application/json',
  };
  Map<String, int> data = {'kakaoId': kakaoId};
  String url = 'https://j8a708.p.ssafy.io/api/v1/user/login';
  http.Response response = await http.post(Uri.parse(url),
      body: json.encode(data), headers: headers);
  final int statusCode = response.statusCode;

  final decodedBody = utf8.decode(response.bodyBytes);
  final result = jsonDecode(decodedBody);
  return result;
}

Future postRobotAdd(int userId, String nickname, int robotNumber) async {
  Map<String, String> headers = {
    'Content-Type': 'application/json',
    'Accept': 'application/json',
  };
  Map<String, dynamic> data = {
    "userId": userId,
    "nickname": nickname,
    "robotNumber": robotNumber
  };
  String url = 'https://j8a708.p.ssafy.io/api/v1/user-homes';
  http.Response response = await http.post(Uri.parse(url),
      body: json.encode(data), headers: headers);
  return response;
}

Future<List<Robot>> getRobots(userid) async {
  final response = await http.get(Uri.parse(
      'https://j8a708.p.ssafy.io/api/v1/user-homes/robots/${userid}'));

  if (response.statusCode == 200) {
    List RobotList = jsonDecode(utf8.decode(response.bodyBytes));

    var Robots = RobotList.map((e) => Robot.fromJson(e)).toList();

    return Robots;
  } else {
    throw Exception('Failed to load album');
  }
}

Future getHomeName(userid, mainhomeid) async {
  String homename = '';
  final response = await http.get(Uri.parse(
      'https://j8a708.p.ssafy.io/api/v1/user-homes/robots/${userid}'));

  if (response.statusCode == 200) {
    List RobotList = jsonDecode(utf8.decode(response.bodyBytes));
    for (var i = 0; i < RobotList.length; i++) {
      RobotList[i]['homeId'] == mainhomeid
          ? homename = RobotList[i]['nickname'].toString()
          : null;
    }
    return homename;
  } else {
    throw Exception('Failed to load album');
  }
}

class Robot {
  final int? userHomeId;
  final int? userId;
  final int? homeId;
  final String? nickname;
  final int? robotNumber;
  final bool? alarm;

  Robot(
      {this.userHomeId,
      this.userId,
      this.homeId,
      this.nickname,
      this.robotNumber,
      this.alarm});

  factory Robot.fromJson(Map<String, dynamic> json) {
    // print(json["scheduleTime"].runtimeType);
    // print(json["content"].runtimeType);
    // print(json["startDate"].runtimeType);
    return Robot(
        userHomeId: json["userHomeId"],
        userId: json["userId"],
        homeId: json["homeId"],
        nickname: json["nickname"],
        robotNumber: json["robotNumber"],
        alarm: json["alarm"]);
  }
}
