import 'package:dolbot/component/home/main_appliance.dart';
import 'package:dolbot/component/home/weather_screen.dart';
import 'package:flutter/material.dart';

class HomeScreen extends StatelessWidget {
  const HomeScreen({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
        child: Column(
      children: [WeatherScreen(), Expanded(child: MainAppliance())],
    ));
  }
}
