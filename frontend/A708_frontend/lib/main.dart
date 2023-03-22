import 'dart:convert';

import 'package:dolbot/database/drift_database.dart';
import 'package:dolbot/screen/home_screen.dart';
import 'package:dolbot/screen/main/main_screen.dart';
import 'package:dolbot/sockect/sockect.dart';
import 'package:drift/drift.dart';
import 'package:flutter/material.dart';
import 'package:intl/date_symbol_data_local.dart';
import 'package:get_it/get_it.dart';

const DEFALUT_COLORS = [
  //빨강
  'F44336',
  //주황
  'FF9800',
  //노랑
  'FFEB3B',
  //초록
  'FCAF50',
  //파랑
  '2196F3',
  //남
  '3F51B5',
  //보라
  '9C27B0'
];

void main() async {

  WidgetsFlutterBinding.ensureInitialized();
  await initializeDateFormatting();
  final database = LocalDatabase();
  final colors = await database.getCategoryColors();

  GetIt.I.registerSingleton<LocalDatabase>(database);
  if (colors.isEmpty) {
    for (String hexCode in DEFALUT_COLORS) {
      await database.createCategoryColor(
        CategoryColorsCompanion(
          hexCode: Value(hexCode),
        ),
      );
    }
  }
  runApp(
    MaterialApp(
        theme: ThemeData(fontFamily: 'Samsung'),
        debugShowCheckedModeBanner: false,
        initialRoute: '/',
        routes: {
          '/': (context) => MainScreen(),
          // '/intro':(context) => IntroScreen(),
          // '/calendar': (context) => CalendarScreen(),
          // '/log':(context)=>LogScreen(),
        }),
  );
}
