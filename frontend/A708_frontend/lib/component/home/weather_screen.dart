import 'dart:convert';
import 'package:socket_io_client/socket_io_client.dart' as IO;
import 'dart:async';
import 'package:dolbot/component/alert/toast.dart';
import 'package:dolbot/const/text_style.dart';
import 'package:dolbot/sockect/sockect.dart';
import 'package:flutter/material.dart';

class WeatherScreen extends StatefulWidget {
  WeatherScreen({
    Key? key,
  }) : super(key: key);

  @override
  State<WeatherScreen> createState() => _WeatherScreenState();
}

class _WeatherScreenState extends State<WeatherScreen> {
  final _outTempStreamController = StreamController<int>();
  final _inTempStreamController = StreamController<int>();
  StreamSocket streamSocket = StreamSocket();
  IO.Socket? socket;
  int? out_temp;
  int? in_temp;
  int? in_hum;
  int? out_hum;
  String? environment;
  @override
  void dispose() {
    super.dispose();
    _outTempStreamController.close();
    _inTempStreamController.close();
  }

  @override
  void initState() {
    super.initState();
    // Connect to the socket server
    IO.Socket socket = IO.io('http://3.36.67.119:8081',
        IO.OptionBuilder().setTransports(['websocket']).build());
    Map<String, dynamic> weather = {
      'type': 'user',
      'id': 1,
      'to': 1,
      'message': 'True'
    };
    socket.onConnect((_) {
      String WEATHER = jsonEncode(weather);
      socket.emit('init_user', WEATHER);
      print('connect');
      // socket.emit('user_message', message);
    });
    // Listen for weather updates
    //home_status
    socket.on('weather_status', (data) {
      var weatherData = jsonDecode(data);
      var message = weatherData['message'];
      print('파싱$weatherData //// $out_temp /// ${weatherData.runtimeType}');
      Map<String, dynamic> weatherMap = json.decode(data);
      print(weatherMap);
      print(weatherMap.runtimeType);
      toast(context, '현재 날씨 ${environment}');
      if (message['out_temp'] != null) {
        setState(() {
          out_temp = message['out_temp'];
          in_temp = message['in_temp'];
          out_hum = message['out_hum'];
          in_hum = message['in_hum'];
          environment = message['environment'];
        });
        _outTempStreamController.add(out_temp!);
        _inTempStreamController.add(in_temp!);
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    setState(() {});
    return Container(
        child: Column(
      mainAxisAlignment: MainAxisAlignment.center,
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        StreamBuilder<int>(
            stream: _outTempStreamController.stream,
            builder: (context, snapshot) {
              if (snapshot.hasData) {
                return OutterWeather(
                  out_hum: out_hum,
                  out_temp: snapshot.data!,
                );
              }
              else {
                return OutterWeather(
                  out_hum: 0,
                  out_temp: 0,
                );
              }
            }),
        StreamBuilder<int>(
            stream: _inTempStreamController.stream,
            builder: (context, snapshot) {
              if (snapshot.hasData) {
                return InnerWeather(
                  in_temp: snapshot.data!,
                  out_temp: out_temp,
                  in_hum: in_hum,
                );
              } else {
                return InnerWeather(
                in_temp: 0,
                out_temp: 0,
                in_hum: 0,
                );
              }
            }),
      ],
    ));
  }
}

class OutterWeather extends StatelessWidget {
  int? out_temp;
  int? out_hum;
  OutterWeather({required this.out_temp, required this.out_hum, Key? key})
      : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16.0, vertical: 8.0),
      child: Container(
          height: MediaQuery.of(context).size.height / 10,
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(16),
            color: Colors.blue[100],
          ),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            crossAxisAlignment: CrossAxisAlignment.end,
            children: [
              Image.asset(
                'asset/img/cloud.png',
                width: MediaQuery.of(context).size.width / 18 * 2,
                height: MediaQuery.of(context).size.height / 10 * 2,
              ),
              Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  WeatherText(content: '외부 온도'),
                  WeatherText(temp: out_temp),
                ],
              ),
              Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  WeatherText(content: '외부 습도'),
                  WeatherText(temp: out_hum),
                ],
              )
            ],
          )),
    );
  }
}

class InnerWeather extends StatelessWidget {
  int? out_temp;
  int? in_temp;
  int? in_hum;
  InnerWeather(
      {required this.in_hum,
      required this.out_temp,
      required this.in_temp,
      Key? key})
      : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16.0, vertical: 8.0),
      child: Container(
        height: MediaQuery.of(context).size.height / 15,
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(16),
          color: Colors.blue,
        ),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: [
            Text('실내'),
            Text('${in_temp}'),
            Text('${in_hum}'),
          ],
        ),
      ),
    );
  }
}
