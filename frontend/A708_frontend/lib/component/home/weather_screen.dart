import 'package:dolbot/const/text_style.dart';
import 'package:flutter/material.dart';

class WeatherScreen extends StatelessWidget {
  const WeatherScreen({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          OutterWeather(),
          InnerWeather(),
        ],
      ),
    );
  }
}

class OutterWeather extends StatelessWidget {
  const OutterWeather({Key? key}) : super(key: key);
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
                WeatherText(content: '외부 기온'),
                WeatherText(content: '20도'),
              ],
            ),
            Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                WeatherText(content: '외부 습도'),
                WeatherText(content: '20도'),
              ],
            )
          ],
        ),
      ),
    );
  }
}

class InnerWeather extends StatelessWidget {
  const InnerWeather({Key? key}) : super(key: key);

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
          children: [Text('실내'), Text(' 20도'), Text(' 40%')],
        ),
      ),
    );
  }
}
