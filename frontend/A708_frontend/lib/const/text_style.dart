import 'package:flutter/material.dart';

class WeatherText extends StatelessWidget {
  final String content;
  const WeatherText({required this.content, Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          crossAxisAlignment: CrossAxisAlignment.center,

          children: [
            Text(
      content,
      style: TextStyle(color: Colors.white, fontWeight: FontWeight.w700),
    ),
          ],
        ));
  }
}
