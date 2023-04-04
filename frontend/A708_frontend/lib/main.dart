import 'dart:convert';
import 'package:dolbot/screen/home_screen.dart';
import 'package:dolbot/screen/main/main_screen.dart';
import 'package:dolbot/screen/splash_screen.dart';
import 'package:dolbot/sockect/sockect.dart';
import 'package:drift/drift.dart';
import 'package:flutter/material.dart';
import 'package:flutter_native_splash/flutter_native_splash.dart';
import 'package:intl/date_symbol_data_local.dart';
import 'package:get_it/get_it.dart';
import 'package:kakao_flutter_sdk_user/kakao_flutter_sdk_user.dart';

void main() async {

  // WidgetsFlutterBinding.ensureInitialized();
  // // WidgetsBinding widgetsBinding = WidgetsFlutterBinding.ensureInitialized();
  KakaoSdk.init(nativeAppKey: '2851dab63468fc5098929a40e856462c');

  // FlutterNativeSplash.preserve(widgetsBinding: widgetsBinding);
  // FlutterNativeSplash.remove();
  runApp(
    MaterialApp(
        theme: ThemeData(fontFamily: 'Samsung'),
        debugShowCheckedModeBanner: false,
        initialRoute: '/',
        routes: {
          '/': (context) => SplashScreen(),
          // '/home': (context) => MainScreen(),
          // '/intro':(context) => IntroScreen(),
          // '/calendar': (context) => CalendarScreen(),
          // '/log':(context)=>LogScreen(),
        }),
  );
}
