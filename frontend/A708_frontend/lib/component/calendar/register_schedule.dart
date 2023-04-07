import 'package:flutter/material.dart';
import 'package:intl/intl.dart';

class RegisterSchedule extends StatefulWidget {
  const RegisterSchedule({Key? key}) : super(key: key);

  @override
  State<RegisterSchedule> createState() => _RegisterScheduleState();
}

class _RegisterScheduleState extends State<RegisterSchedule> {
  final GlobalKey<FormState> formKey = GlobalKey();
  DateTime today = DateTime(
    DateTime.now().year,
    DateTime.now().month,
    DateTime.now().day,
  );
  int _year = DateTime.now().year.toInt();
  int _month = DateTime.now().month.toInt();
  int _day = DateTime.now().day.toInt();

  @override
  Widget build(BuildContext context) {
    final bottomInset = MediaQuery.of(context).viewInsets.bottom;
    return GestureDetector(
      onTap: () {
        FocusScope.of(context).requestFocus(FocusNode());
      },
      child: Container(
        color: Colors.white,
        height: MediaQuery.of(context).size.height / 2 + bottomInset,
        child: Form(
          key: formKey,
          child: Column(
            children: [
              Text('${_year}년 ${_month}월 ${_day}일의 일정'),
              Row(
                children: [],
              )
            ],
          ),
        ),
      ),
    );
  }
}
