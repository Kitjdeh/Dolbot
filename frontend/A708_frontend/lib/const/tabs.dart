import 'package:dolbot/screen/calendar_screen.dart';
import 'package:dolbot/screen/cctv_screen.dart';
import 'package:dolbot/screen/home_screen.dart';
import 'package:dolbot/component/log/daily_log.dart';
import 'package:dolbot/screen/log_screen.dart';
import 'package:dolbot/screen/menu_screen.dart';
import 'package:flutter/material.dart';

class TabInfo {
  final IconData icon;
  final String label;
  final String location;

  const TabInfo({
    required this.icon,
    required this.label,
    required this.location,
  });
}

const TABS = [
  TabInfo(icon: Icons.home, label: '홈', location: 'HomeScreen'),
  TabInfo(
      icon: Icons.calendar_month_sharp,
      label: '일정',
      location: 'CalendarScreen'),
  TabInfo(icon: Icons.doorbell_outlined, label: '로그', location: 'LogScreen'),
  TabInfo(icon: Icons.camera_indoor, label: 'CCTV', location: 'CctvScreen'),
  TabInfo(icon: Icons.menu, label: '메뉴', location: 'MenuScreen'),
];
// final WIDGETS = [
//   HomeScreen(),
//   CalendarScreen(),
//   LogScreen(),
//   CctvScreen(),
//   MenuScreen()
// ];
