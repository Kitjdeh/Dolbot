import 'dart:convert';
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
  IO.Socket socket = IO.io('http://3.36.67.119:8080',
      IO.OptionBuilder().setTransports(['websocket']).build());
  socket.onConnect((_) {
    socket.emit('init_user', message);
    print('connect');
    socket.emit('user_message', message);
  });
  socket.on('user_message', (data) => {print(data), streamSocket.addResponse});
  socket.onDisconnect((_) => print('disconnect'));
  print('ffff${streamSocket.addResponse}');
}

Map<String, dynamic> toJson = {
  't': 'user',
  'id': 15,
  'to': 1,
  'message': 'connect'
};

String jsonString = jsonEncode(toJson);
String jStr = "{ 't' : 'user', 'id' : 15, 'to' : 1, 'message' : 'appHello'  }";

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
