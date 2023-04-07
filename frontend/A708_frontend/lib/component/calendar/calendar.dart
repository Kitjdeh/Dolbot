import 'package:flutter/material.dart';
import 'package:table_calendar/table_calendar.dart';

class Calendar extends StatelessWidget {
  final DateTime? selectedDay;
  final DateTime focusedDay;
  final OnDaySelected onDaySelected;
  const Calendar(
      {required this.selectedDay,
      required this.onDaySelected,
      required this.focusedDay,
      Key? key})
      : super(key: key);

  @override
  Widget build(BuildContext context) {
    final defaultBoxDeco =
        BoxDecoration(borderRadius: BorderRadius.circular(30.0));
    final defaultTextStyle = TextStyle(
      color: Colors.grey[600],
      fontWeight: FontWeight.w700,
    );
    return TableCalendar(
      locale: 'ko_KR',
      focusedDay: focusedDay,
      firstDay: DateTime(1800),
      lastDay: DateTime(2040),
      headerStyle: HeaderStyle(
        formatButtonVisible: false,
        titleCentered: true,
        titleTextStyle: TextStyle(
          fontWeight: FontWeight.w700,
          fontSize: 16.0,
        ),
      ),
      calendarStyle: CalendarStyle(
        isTodayHighlighted: false,
        defaultDecoration: defaultBoxDeco,
        weekendDecoration: defaultBoxDeco,
        selectedDecoration: BoxDecoration(
          color: Colors.blue[100],
          borderRadius: BorderRadius.circular(16.0),
        ),
        outsideDecoration: BoxDecoration(shape: BoxShape.rectangle),
        defaultTextStyle: defaultTextStyle,
        weekendTextStyle: defaultTextStyle,
        selectedTextStyle: defaultTextStyle.copyWith(
          color: Colors.white,
        ),
      ),
      onDaySelected: onDaySelected,
      selectedDayPredicate: (DateTime date) {
        if (selectedDay == null) {
          return false;
        }
        return date.year == selectedDay!.year &&
            date.month == selectedDay!.month &&
            date.day == selectedDay!.day;
      },
    );
  }
}
