import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:socket_io_client/socket_io_client.dart' as IO;
import 'dart:async';

class MenuScreen extends StatefulWidget {
  const MenuScreen({Key? key}) : super(key: key);

  @override
  State<MenuScreen> createState() => _MenuScreenState();
}

class _MenuScreenState extends State<MenuScreen> {
  List<int> numbers = List.generate(100, (index) => index);
  @override
  Widget build(BuildContext context) {
    print('111');
    connectAndListen();
    return Scaffold(
        appBar: AppBar(
          title: Text('메뉴바'),
        ),
        body: BuildWithSocketStream());
  }

  Widget ListViewContainer() {
    return Container(
      child: ReorderableListView.builder(
        itemBuilder: (context, index) {
          return renderContainer(
              color: rainbowColors[numbers[index] % rainbowColors.length],
              index: numbers[index]);
        },
        itemCount: numbers.length,
        onReorder: (int oldIndex, int newIndex) {
          setState(() {
            if (oldIndex < newIndex) {
              newIndex -= 1;
            }
            final item = numbers.removeAt(oldIndex);
            numbers.insert(newIndex, item);
          });
        },
      ),
    );
  }

  Widget renderContainer({
    required Color color,
    required int index,
    double? height,
  }) {
    return Container(
        key: Key(index.toString()),
        height: height == null ? 100 : height,
        color: color,
        child: Center(
          child: Text(
            index.toString(),
            style: TextStyle(
                color: Colors.white, fontWeight: FontWeight.w700, fontSize: 30),
          ),
        ));
  }
}

const rainbowColors = [
  Colors.red,
  Colors.orange,
  Colors.yellow,
  Colors.green,
  Colors.blue,
  Colors.indigo,
  Colors.purple
];

Map<String, dynamic> toJson = {
  't': 'user',
  'id': 15,
  'to': 1,
  'message': 'connect'
};
Map<String, dynamic> twoJson = {
  't': 'user',
  'id': 15,
  'to': 1,
  'message': '퇴실33분전'
};
String jsonString = jsonEncode(toJson);
String jsonStringtwo = jsonEncode(twoJson);
String jStr = "{ 't' : 'user', 'id' : 15, 'to' : 1, 'message' : 'appHello'  }";
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

void connectAndListen() {
  IO.Socket socket = IO.io('http://3.36.67.119:8080',
      IO.OptionBuilder().setTransports(['websocket']).build());

  socket.onConnect((_) {
    print('connect');
    socket.emit('chat_message', jsonString);
    socket.emit('chat_message', jsonStringtwo);
  });
  socket.on('event', (data) => streamSocket.addResponse);

  socket.onDisconnect((_) => print('disconnect'));
  print(streamSocket.addResponse);
}

class BuildWithSocketStream extends StatefulWidget {
  const BuildWithSocketStream({Key? key}) : super(key: key);

  @override
  State<BuildWithSocketStream> createState() => _BuildWithSocketStreamState();
}

class _BuildWithSocketStreamState extends State<BuildWithSocketStream> {
  @override
  Widget build(BuildContext context) {
    return Container(
      child: StreamBuilder(
        stream: streamSocket.getResponse,
        builder: (BuildContext context, AsyncSnapshot<String> snapshot) {
          print(snapshot.data);
          String? message = snapshot.data;
          return Container(
            child: Container(
                child: snapshot.connectionState == ConnectionState.waiting
                    ? CircularProgressIndicator()
                    : Text('${streamSocket.getResponse}')),
          );
        },
      ),
    );
  }
}
