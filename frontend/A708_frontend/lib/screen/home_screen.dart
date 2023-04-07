import 'package:dolbot/component/alert/toast.dart';
import 'package:dolbot/component/home/main_appliance.dart';
import 'package:dolbot/component/home/weather_screen.dart';
import 'package:dolbot/sockect/sockect.dart';
import 'package:flutter/material.dart';
import 'package:socket_io_client/socket_io_client.dart' as IO;
import 'dart:async';
import 'dart:convert';

class HomeScreen extends StatefulWidget {
  IO.Socket socket;
  int? out_temp;
  int? in_temp;
  int? in_hum;
  int? out_hum;
  int? air;
  String? environment;
  Map<dynamic, dynamic>? aplicancestatus;
  HomeScreen(
      {required this.socket,
      this.in_hum,
      this.out_hum,
      this.out_temp,
      this.in_temp,
      this.environment,
      this.air,
      this.aplicancestatus,
      Key? key})
      : super(key: key);

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  // StreamSocket streamSocket = StreamSocket();

  // final _out_humStreamController = StreamController<int>();
  // int? out_temp;
  // int? in_temp;
  // int? in_hum;
  // int? out_hum;
  // String? environment;
  @override
  void initState() {
    super.initState();
    // Connect to the socket server
    // IO.Socket socket = IO.io('http://j8a708.p.ssafy.io:8081',
    //     IO.OptionBuilder().setTransports(['websocket']).setTimeout(1000000000000).build());
    // Map<String, dynamic> weather = {
    //   'type': 'user',
    //   'id': 3,
    //   'to': 708002,
    //   'message': 'True'
    // };
    // String WEATHER = jsonEncode(weather);
    // socket.onConnect((_) {
    //
    //   socket.emit('init_user', WEATHER);
    //   print('connect');
    //   // socket.emit('user_message', message);
    // });}
    // Listen for weather updates
    //home_status
  }

  // @override
  // void dispose() {
  //   if (socket != null) {
  //     socket!.disconnect();
  //   }
  //
  //   super.dispose();
  // }

  @override
  Widget build(BuildContext context) {
    // setState(() {
    //   if (widget.out_temp != null) {
    //     widget.out_temp = _out_temp;
    //     widget.in_temp = _in_temp;
    //     widget.in_hum = _in_hum;
    //     widget.out_hum = _out_hum;
    //     widget.environment = _environment;
    //   }
    // });
    return SafeArea(
      child: Container(
          child: Column(
        children: [
          WeatherScreen(
              out_temp: widget.out_temp,
              in_temp: widget.in_temp,
              in_hum: widget.in_hum,
              out_hum: widget.out_hum,
              environment: widget.environment),
          Expanded(
              child: MainAppliance(
                air:widget.air,
            aplicancestatus: widget.aplicancestatus,
          ))
        ],
      )),
    );
  }
}
