import 'package:dolbot/screen/home_screen.dart';
import 'package:dolbot/screen/splash_screen.dart';
import 'package:flutter/material.dart';
import 'package:kakao_flutter_sdk_user/kakao_flutter_sdk_user.dart';

class KakaoLogin implements SocialLogin {
  @override
  Future<bool> login() async {
    try {
      bool isInstalled = await isKakaoTalkInstalled();
      print('1234$isInstalled');
      if (isInstalled) {
        try {
          await UserApi.instance.loginWithKakaoTalk();
          print('로그인');
          return true;
        } catch (e) {
          print('login직전');
          return false;
        }
      } else {
        try {
          await UserApi.instance.loginWithKakaoAccount();
          print('isKakato install');
          return true;
        } catch (e) {
          print('1$e');
                    return false;
        }
      }
    } catch (e) {
      print('2');
      return false;
    }
  }

  @override
  Future<bool> logout() async {
    try {
      await UserApi.instance.unlink();
      return true;
    } catch (error) {
      return false;
    }
  }
}
