import 'package:flutter/material.dart';

class WeatherText extends StatelessWidget {
  final int? temp;
  final String? content;
  const WeatherText({this.temp, this.content, Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
        child: Column(
      mainAxisAlignment: MainAxisAlignment.center,
      crossAxisAlignment: CrossAxisAlignment.center,
      children: [
        Text(temp != null ? '$temp도' : '$content',
            style: TextStyle(color: Colors.white, fontWeight: FontWeight.w700)),
      ],
    ));
  }
}
