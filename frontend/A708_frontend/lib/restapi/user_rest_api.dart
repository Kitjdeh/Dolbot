import 'dart:convert';
import 'package:http/http.dart' as http;


Future postLogin(kakaoId) async {
  Map<String, String> headers = {
    'Content-Type': 'application/json',
    'Accept': 'application/json',
  };
  Map<String, int> data = {'kakaoId' : kakaoId};
  String url = 'https://j8a708.p.ssafy.io/api/v1/user/login';
  http.Response response = await http.post(Uri.parse(url),
      body: json.encode(data), headers: headers);
  final int statusCode = response.statusCode;
  print(statusCode);
  final decodedBody = utf8.decode(response.bodyBytes);
  final result = jsonDecode(decodedBody);
  return result;
}

Future postRobotAdd(int userId,String nickname,int robotNumber) async {
  Map<String, String> headers = {
    'Content-Type': 'application/json',
    'Accept': 'application/json',
  };
  Map<String, dynamic> data = { "userId": userId,
    "nickname": nickname,
    "robotNumber": robotNumber};
  String url = 'https://j8a708.p.ssafy.io/api/v1/user-homes';
  http.Response response = await http.post(Uri.parse(url),
      body: json.encode(data), headers: headers);
  final int statusCode = response.statusCode;
  print(statusCode);
  final decodedBody = utf8.decode(response.bodyBytes);
  final result = jsonDecode(decodedBody);
  return result;
}
