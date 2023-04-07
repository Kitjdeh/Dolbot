import 'dart:convert';
import 'package:dolbot/component/alert/toast.dart';
import 'package:flutter/material.dart';
import 'package:socket_io_client/socket_io_client.dart' as IO;
import 'dart:async';

StreamSocket streamSocket = StreamSocket();

// STEP1:  Stream setup
class StreamSocket {
  final _socketResponse = StreamController<String>();

  void Function(String) get addResponse => _socketResponse.sink.add;

  Stream<String> get getResponse => _socketResponse.stream;

  void dispose() {
    _socketResponse.close();
  }
}

void connectAndListen(message) {
  IO.Socket socket = IO.io('https://j8a708.p.ssafy.io/socket',
      IO.OptionBuilder().setTransports(['websocket']).build());
  socket.onConnect((_) {
    socket.emit('init_user', message);
    Map<String, dynamic> weather = {
      'type': 'user',
      'id': 1,
      'to': 708001,
      'message': 'True'
    };
    print('123123');
    String request_weather = jsonEncode(weather);
    socket.emit('user_message', request_weather);
    print('connectdddddddddddd');
    // socket.emit('user_message', message);
  });
  socket.on('weather_status', (data) {
    print('데이터 형식${data}');
    var weatherData = jsonDecode(data);
    var environment = weatherData['message'];
    int? out_temp = environment['out_temp'];
    print('파싱$weatherData //// $out_temp /// ${weatherData.runtimeType}');
    Map<String, dynamic> weatherMap = json.decode(data);
  });
  socket.on('weather_status', (data) => streamSocket.addResponse);
  socket.onDisconnect((_) => print('disconnect'));
  // print('${streamSocket.addResponse}');
}

void SendMessage(status,message) {
  IO.Socket socket = IO.io('http://j8a708.p.ssafy.io:8081',
      IO.OptionBuilder().setTransports(['websocket']).build());
  socket.emit(status, message);
  print('send success');
}

Map<String, dynamic> toJson = {
  't': 'user',
  'id': 1,
  'to': 708001,
  'message': 'connect'
};
// void SendNewMessage(status,message) {
//   IO.Socket socket = IO.io('http://j8a708.p.ssafy.io:8081',
//       IO.OptionBuilder().setTransports(['websocket']).build());
//   socket.emit(status, message);
//   print('send success');
// }

String jsonString = jsonEncode(toJson);
String jStr = "{ 't' : 'user', 'id' : 1, 'to' : 1, 'message' : 'appHello'  }";

class message {
  final int room;
  final String device;
  final String status;
  final Map<String, dynamic>? meta;
  const message(
      {required this.room,
      required this.device,
      required this.status,
      this.meta});
}

class JStr {
  final String type;
  final int id;
  final int to;
  final Map<String, dynamic> message;

  const JStr({
    required this.type,
    required this.id,
    required this.to,
    required this.message,
  });
}

class Weather {
  final String type;
  final int id;
  final int to;
  final Map<String, dynamic> message;

  const Weather({
    required this.type,
    required this.id,
    required this.to,
    required this.message,
  });
}

// class BuildWithSocketStream extends StatefulWidget {
//   const BuildWithSocketStream({Key? key}) : super(key: key);
//
//   @override
//   State<BuildWithSocketStream> createState() => _BuildWithSocketStreamState();
// }
//
// class _BuildWithSocketStreamState extends State<BuildWithSocketStream> {
//   @override
//   Widget build(BuildContext context) {
//     return Container(
//       child: StreamBuilder(
//         stream: streamSocket.getResponse,
//         builder: (BuildContext context, AsyncSnapshot<String> snapshot) {
//           print(snapshot.data);
//           String? message = snapshot.data;
//           return Container(
//             child: Container(
//                 child: snapshot.connectionState == ConnectionState.waiting
//                     ? CircularProgressIndicator()
//                     : Text('${streamSocket.getResponse}')),
//           );
//         },
//       ),
//     );
//   }
// }
