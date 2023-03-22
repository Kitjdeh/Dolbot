import 'package:dolbot/component/log/log_box.dart';
import 'package:dolbot/const/data.dart';
import 'package:flutter/material.dart';

class LogScreen extends StatelessWidget {
  LogScreen({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return SingleChildScrollView(
      child: Column(
        children: [
          Image.asset('asset/img/smile.gif'),
          renderSimple(),
        ],
      ),
    );
  }

  Widget renderSimple() {
    return Container(
      decoration: BoxDecoration(color: Colors.white),
      child: Column(
        children: [
          Column(
              children: LOGINFO
                  .map((e) => LogBox(
                      index: e.index,
                      location: e.location,
                      content: e.content,
                      endtime: e.endtime))
                  .toList())
        ],
      ),
    );
  }
}
