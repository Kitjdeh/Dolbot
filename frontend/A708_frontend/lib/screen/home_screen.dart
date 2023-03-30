import 'package:dolbot/component/home/main_appliance.dart';
import 'package:dolbot/component/home/weather_screen.dart';
import 'package:dolbot/sockect/sockect.dart';
import 'package:flutter/material.dart';
import 'package:socket_io_client/socket_io_client.dart' as IO;
import 'dart:async';
import 'dart:convert';

class HomeScreen extends StatefulWidget {
  const HomeScreen({Key? key}) : super(key: key);

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  StreamSocket streamSocket = StreamSocket();
  @override
  void initState() {
    super.initState();
    // Connect to the socket server
    IO.Socket socket = IO.io('http://3.36.67.119:8081',
        IO.OptionBuilder().setTransports(['websocket']).build());
    socket.onConnect((_) {
      // socket.emit('init_user', message);
      Map<String, dynamic> weather = {
        'type': 'user',
        'id': 1,
        'to': 1,
        'message': 'True'
      };

      String request_weather = jsonEncode(weather);
      socket.emit('user_message', request_weather);
      print('connect');
      // socket.emit('user_message', message);
    });
    // Listen for weather updates
    socket.on('weather_status', (data) {
      var weatherData = jsonDecode(data);
      var message = weatherData['message'];
      int? out_temp = message['out_temp'];
      int? in_temp = message['int_temp'];
      int? out_hum = message['out_hum'];
      String? environment = message['enviroment'];
      print('파싱$weatherData //// $out_temp /// ${weatherData.runtimeType}');
      Map<String, dynamic> weatherMap = json.decode(data);
      print(weatherMap);
      print(weatherMap.runtimeType);
      setState(() {
        Map<String, dynamic> weatherstatus = message;
      });

    });
  }

  @override
  Widget build(BuildContext context) {
    return SafeArea(
      child: Container(
          child: Column(
        children: [WeatherScreen(), Expanded(child: MainAppliance())],
      )),
    );
  }
}
