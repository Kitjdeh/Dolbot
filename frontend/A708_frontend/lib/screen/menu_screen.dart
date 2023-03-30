import 'dart:convert';
import 'package:dolbot/component/log/log_box.dart';
import 'package:dolbot/sockect/sockect.dart';
import 'package:http/http.dart' as http;
import 'package:flutter/material.dart';
import 'package:socket_io_client/socket_io_client.dart' as IO;
import 'dart:async';

class MenuScreen extends StatefulWidget {
  const MenuScreen({Key? key}) : super(key: key);

  @override
  State<MenuScreen> createState() => _MenuScreenState();
}

class _MenuScreenState extends State<MenuScreen> {
  // late Future<List<Schedule>>? ScheduleList;

  // @override
  // void initState() {
  //   super.initState();
  //   ScheduleList = fetchAlbum();
  // }

  @override
  Widget build(BuildContext context) {
    connectAndListen('asdf');
    return Scaffold(
        appBar: AppBar(
          title: Text('메뉴바'),
        ),
        body: Center(
            child: Container(
                child: Column(
          children: [
            Text('망'),
            Text('jjj'),
          ],
        ))

            // By default, show a loading spinner.

            ));
  }

  // Widget ListViewContainer() {
  //   return Container(
  //     child: ReorderableListView.builder(
  //       itemBuilder: (context, index) {
  //         return renderContainer(
  //             color: rainbowColors[numbers[index] % rainbowColors.length],
  //             index: numbers[index]);
  //       },
  //       itemCount: numbers.length,
  //       onReorder: (int oldIndex, int newIndex) {
  //         setState(() {
  //           if (oldIndex < newIndex) {
  //             newIndex -= 1;
  //           }
  //           final item = numbers.removeAt(oldIndex);
  //           numbers.insert(newIndex, item);
  //         });
  //       },
  //     ),
  //   );
  // }

  // Widget renderContainer({
  //   required Color color,
  //   required int index,
  //   double? height,
  // }) {
  //   return Container(
  //       key: Key(index.toString()),
  //       height: height == null ? 100 : height,
  //       color: color,
  //       child: Center(
  //         child: Text(
  //           index.toString(),
  //           style: TextStyle(
  //               color: Colors.white, fontWeight: FontWeight.w700, fontSize: 30),
  //         ),
  //       ));
  // }
}
//
// Future<List<Schedule>> fetchAlbum() async {
//   final response = await http.get(Uri.parse(
//       'http://j8a708.p.ssafy.io:8080/api/v1/schedule-info/1?localDate=2023-03-23'));
//   print('111${response.body}');
//   if (response.statusCode == 200) {
//     List dataList = jsonDecode(response.body);
//     print('222$dataList');
//     var Schedules = dataList.map((e) => Schedule.fromJson(e)).toList();
//     print('3333${Schedules}');
//     return Schedules;
//   } else {
//     throw Exception('Failed to load album');
//   }
// }
//
// class Schedule {
//   final int? scheduleId;
//   final int? homeId;
//   final int? scheduleTime;
//   final String? content;
//   final String? startDate;
//   final String? endDate;
//
//   Schedule(
//       {this.scheduleId,
//       this.homeId,
//       this.scheduleTime,
//       this.content,
//       this.endDate,
//       this.startDate});
//
//   factory Schedule.fromJson(Map<String, dynamic> json) {
//     print(json["scheduleTime"].runtimeType);
//     print(json["content"].runtimeType);
//     print(json["startDate"].runtimeType);
//     return Schedule(
//         scheduleId: json["scheduleId"],
//         homeId: json["homeId"],
//         scheduleTime: json["scheduleTime"],
//         content: json["content"],
//         startDate: json["startDate"],
//         endDate: json["endDate"]);
//   }
// }

//
// const rainbowColors = [
//   Colors.red,
//   Colors.orange,
//   Colors.yellow,
//   Colors.green,
//   Colors.blue,
//   Colors.indigo,
//   Colors.purple
// ];
//
// Map<String, dynamic> toJson = {
//   't': 'user',
//   'id': 15,
//   'to': 1,
//   'message': 'connect'
// };
// Map<String, dynamic> twoJson = {
//   't': 'user',
//   'id': 15,
//   'to': 1,
//   'message': '퇴실33분전'
// };
// String jsonString = jsonEncode(toJson);
// String jsonStringtwo = jsonEncode(twoJson);
// String jStr = "{ 't' : 'user', 'id' : 15, 'to' : 1, 'message' : 'appHello'  }";
// StreamSocket streamSocket = StreamSocket();
// // STEP1:  Stream setup
// class StreamSocket {
//   final _socketResponse = StreamController<String>();
//
//   void Function(String) get addResponse => _socketResponse.sink.add;
//
//   Stream<String> get getResponse => _socketResponse.stream;
//
//   void dispose() {
//     _socketResponse.close();
//   }
// }
//
// void connectAndListen() {
//   IO.Socket socket = IO.io('http://3.36.67.119:8080',
//       IO.OptionBuilder().setTransports(['websocket']).build());
//
//   socket.onConnect((_) {
//     print('connect');
//     socket.emit('chat_message', jsonString);
//     socket.emit('chat_message', jsonStringtwo);
//   });
//   socket.on('event', (data) => streamSocket.addResponse);
//
//   socket.onDisconnect((_) => print('disconnect'));
//   print(streamSocket.addResponse);
// }
//
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
