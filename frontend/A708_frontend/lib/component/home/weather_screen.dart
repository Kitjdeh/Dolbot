import 'dart:convert';
import 'package:dolbot/const/colors.dart';
import 'package:socket_io_client/socket_io_client.dart' as IO;
import 'dart:async';
import 'package:dolbot/component/alert/toast.dart';
import 'package:dolbot/const/text_style.dart';
import 'package:dolbot/sockect/sockect.dart';
import 'package:flutter/material.dart';

class WeatherScreen extends StatefulWidget {
  int? out_temp;
  int? in_temp;
  int? in_hum;
  int? out_hum;
  String? environment;
  WeatherScreen({
    this.in_hum,
    this.out_hum,
    this.out_temp,
    this.in_temp,
    this.environment,
    Key? key,
  }) : super(key: key);

  @override
  State<WeatherScreen> createState() => _WeatherScreenState();
}

class _WeatherScreenState extends State<WeatherScreen> {
  final _outTempStreamController = StreamController<int>();
  final _inTempStreamController = StreamController<int>();
  final _out_humStreamController = StreamController<int>();
  final _in_humStreamController = StreamController<int>();
  // StreamSocket streamSocket = StreamSocket();
  // IO.Socket? socket;
  int? _out_temp;
  int? _in_temp;
  int? _in_hum;
  int? _out_hum;
  String? _environment;
  // @override
  // void dispose() {
  //   if (socket != null) {
  //     socket!.disconnect();
  //   }
  //   _outTempStreamController.close();
  //   _inTempStreamController.close();
  //   _out_humStreamController.close() ;
  //   _in_humStreamController.close() ;
  //   super.dispose();
  // }
  StreamSubscription<int>? _outTempSubscription;
  @override
  Widget build(BuildContext context) {
    // setState(() {
    print('outtemp ////${widget.out_temp} ');
    if (mounted && widget.out_temp != null) {
      _out_temp = widget.out_temp;
      _in_temp = widget.in_temp;
      _in_hum = widget.in_hum;
      _out_hum = widget.out_hum;
      _environment = widget.environment;
      print('weatherdata수령${_out_hum}');
    }

    // });
    print('weatherbox${widget.out_temp}');
    return Container(
        child: Column(
      mainAxisAlignment: MainAxisAlignment.center,
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        OutterWeather(
          out_hum: _out_hum,
          out_temp: _out_temp,
        ),
        InnerWeather(
          in_hum: _in_hum,
          in_temp: _in_temp,
        )

        // StreamBuilder<int>(
        //     stream: _inTempStreamController.stream,
        //     builder: (context, snapshot) {
        //       if (snapshot.hasData) {
        //         return InnerWeather(
        //           in_temp: snapshot.data!,
        //           out_temp: widget.out_temp,
        //           in_hum: widget.in_hum,
        //         );
        //       } else {
        //         return InnerWeather(
        //           in_temp: 0,
        //           out_temp: 0,
        //           in_hum: 0,
        //         );
        //       }
        //     }),
      ],
    ));
  }
}

class OutterWeather extends StatefulWidget {
  int? out_temp;
  int? out_hum;

  OutterWeather({required this.out_temp, required this.out_hum, Key? key})
      : super(key: key);

  @override
  State<OutterWeather> createState() => _OutterWeatherState();
}

class _OutterWeatherState extends State<OutterWeather> {
  String? environment;
  final _outtempStreamController = StreamController<int>();
  final _outhumStreamController = StreamController<int>();

  @override
  Widget build(BuildContext context) {
    if (widget.out_hum != null && widget.out_temp != null) {
      _outtempStreamController.add(widget.out_temp ?? 0);
      _outhumStreamController.add(widget.out_hum ?? 0);
    }

    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16.0, vertical: 8.0),
      child: Container(
          height: MediaQuery.of(context).size.height / 10,
          decoration: BoxDecoration(
            borderRadius: BorderRadius.circular(16),
            color: WEATHER,
          ),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            crossAxisAlignment: CrossAxisAlignment.end,
            children: [
              environment == 'Cloudy'
                  ? Image.asset(
                      'asset/img/cloud.png',
                      width: MediaQuery.of(context).size.width / 18 * 2,
                      height: MediaQuery.of(context).size.height / 10 * 2,
                    )
                  : environment == "Snowy"
                      ? Image.asset(
                          'asset/img/weather_snowy.png',
                          width: MediaQuery.of(context).size.width / 18 * 2,
                          height: MediaQuery.of(context).size.height / 10 * 2,
                        )
                      : environment == "Foggy"
                          ? Image.asset(
                              'asset/img/weather_fog.gif',
                              width: MediaQuery.of(context).size.width / 18 * 2,
                              height:
                                  MediaQuery.of(context).size.height / 10 * 2,
                            )
                          : environment == "Stormy"
                              ? Image.asset(
                                  'asset/img/weather_smoke.gif',
                                  width: MediaQuery.of(context).size.width /
                                      18 *
                                      2,
                                  height: MediaQuery.of(context).size.height /
                                      10 *
                                      2,
                                )
                              : environment == "Sunny"
                                  ? Image.asset(
                                      'asset/img/weather_sunny.png',
                                      width: MediaQuery.of(context).size.width /
                                          18 *
                                          2,
                                      height:
                                          MediaQuery.of(context).size.height /
                                              10 *
                                              2,
                                    )
                                  : Image.asset(
                                      'asset/img/cloud.png',
                                      width: MediaQuery.of(context).size.width /
                                          18 *
                                          2,
                                      height:
                                          MediaQuery.of(context).size.height /
                                              10 *
                                              2,
                                    ),
              Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  WeatherText(content: '외부 온도'),
                  StreamBuilder<int>(
                      stream: _outtempStreamController.stream,
                      builder: (context, snapshot) {
                        if (snapshot.hasData) {
                          return WeatherText(temp: snapshot.data);
                        } else {
                          return CircularProgressIndicator();
                        }
                      }),
                ],
              ),
              Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  WeatherText(content: '외부 습도'),
                  StreamBuilder<int>(
                      stream: _outhumStreamController.stream,
                      builder: (context, snapshot) {
                        if (snapshot.hasData) {
                          return WeatherText(temp: snapshot.data);
                        } else {
                          return CircularProgressIndicator();
                        }
                      }),
                ],
              )
            ],
          )),
    );
  }
}

class InnerWeather extends StatefulWidget {
  int? in_temp;
  int? in_hum;
  InnerWeather({required this.in_hum, required this.in_temp, Key? key})
      : super(key: key);

  @override
  State<InnerWeather> createState() => _InnerWeatherState();
}

class _InnerWeatherState extends State<InnerWeather> {
  final _intempStreamController = StreamController<int>();

  final _inhumStreamController = StreamController<int>();

  @override
  Widget build(BuildContext context) {
    if (widget.in_hum != null && widget.in_temp != null) {
      _intempStreamController.add(widget.in_temp ?? 0);
      _inhumStreamController.add(widget.in_hum ?? 0);
    }
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16.0, vertical: 8.0),
      child: Container(
        height: MediaQuery.of(context).size.height / 15,
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(16),
          color: DEEP_BLUE,
        ),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: [
            Text(
              '실내',
              style: TextStyle(color: Colors.white),
            ),
            StreamBuilder<int>(
                stream: _intempStreamController.stream,
                builder: (context, snapshot) {
                  if (snapshot.hasData) {
                    return WeatherText(temp: snapshot.data);
                  } else {
                    return CircularProgressIndicator();
                  }
                }),
            StreamBuilder<int>(
                stream: _inhumStreamController.stream,
                builder: (context, snapshot) {
                  if (snapshot.hasData) {
                    print('asdfasdfasdfasdf${snapshot.data}');
                    return WeatherText(temp: snapshot.data);
                  } else {
                    return CircularProgressIndicator();
                  }
                }),
          ],
        ),
      ),
    );
  }
}
