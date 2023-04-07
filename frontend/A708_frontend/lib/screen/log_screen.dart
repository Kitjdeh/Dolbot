import 'package:dolbot/component/log/daily_log.dart';
import 'package:flutter/material.dart';
import 'package:intl/intl.dart';

class LogScreen extends StatefulWidget {
  const LogScreen({Key? key}) : super(key: key);

  @override
  State<LogScreen> createState() => _LogScreenState();
}

class _LogScreenState extends State<LogScreen> {
  String _selectedDate = DateFormat('yyyy-MM-dd').format(DateTime.now());
  @override
  Widget build(BuildContext context) {
    return Container(
      child: SingleChildScrollView(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            ElevatedButton(
                onPressed: () {
                  _selectDate(context);
                },
                child: Text('$_selectedDate')),
            DailyLog(selectedDate: _selectedDate)
          ],
        ),
      ),
    );
  }

  Future _selectDate(BuildContext context) async {
    final DateTime? selected = await showDatePicker(
        context: context,
        initialDate: DateTime.now(),
        firstDate: DateTime(2010),
        lastDate: DateTime(2030));
    if (selected != null) {
      setState(() {
        _selectedDate = DateFormat('yyyy-MM-dd').format(selected);
      });
    }
  }
}
