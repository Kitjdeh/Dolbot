import 'package:fluttertoast/fluttertoast.dart';
import 'package:flutter/material.dart';

void showToast(content) {
  Fluttertoast.showToast(
      msg: "$content",
      toastLength: Toast.LENGTH_SHORT,
      gravity: ToastGravity.CENTER,
      timeInSecForIosWeb: 3,
      backgroundColor: Colors.blue[100],
      textColor: Colors.blue[100],
      fontSize: 16.0);
}

void toast(context, text) {
  final fToast = FToast();
  fToast.init(context);
  Widget toast = Container(
    padding: const EdgeInsets.symmetric(horizontal: 24.0, vertical: 12.0),
    decoration: BoxDecoration(
      borderRadius: BorderRadius.circular(8.0),
      color: Colors.blue[200],
    ),
    child: Row(
      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
      children: [
        Icon(Icons.check, color: Colors.white),
        SizedBox(width: 10),
        Text(text, style: TextStyle(color: Colors.white)),
      ],
    ),
  );

  fToast.showToast(
      child: toast,
      toastDuration: const Duration(seconds: 2),
      positionedToastBuilder: (context, child) {
        return Stack(
          alignment: Alignment.center,
          children: [
            Positioned(
              child: child,
              bottom: 100,
            ),
          ],
        );
      });
}

void emergency(context, text) {
  final fToast = FToast();
  fToast.init(context);
  Widget toast = Container(
    padding: const EdgeInsets.symmetric(horizontal: 24.0, vertical: 12.0),
    width: 320,
    height: 60,
    decoration: BoxDecoration(
      borderRadius: BorderRadius.circular(8.0),
      color: Colors.red[300],
    ),
    child: Row(
      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
      children: [
        Icon(Icons.check, color: Colors.white),
        SizedBox(width: 10),
        Text(text, style: TextStyle(color: Colors.white)),
      ],
    ),
  );
  fToast.showToast(
      child: toast,
      toastDuration: Duration(seconds: 5),
      positionedToastBuilder: (context, child) {
        return Stack(
          alignment: Alignment.center,
          children: [
            Positioned(
              child: child,
              top: 100,
            ),
          ],
        );
      });
}
