import 'dart:math';

import 'package:dolbot/component/alert/toast.dart';
import 'package:dolbot/const/colors.dart';
import 'package:dolbot/restapi/log_rest_api.dart';
import 'package:dolbot/restapi/user_rest_api.dart';
import 'package:dolbot/screen/home_screen.dart';
import 'package:dolbot/screen/main/main_screen.dart';
import 'package:dolbot/screen/register_screen.dart';
import 'package:flutter/material.dart';
import 'package:kakao_flutter_sdk/kakao_flutter_sdk_user.dart';
import 'package:dolbot/component/login/kakao_login.dart';

class SplashScreen extends StatefulWidget {
  const SplashScreen({Key? key}) : super(key: key);

  @override
  State<SplashScreen> createState() => _SplashScreenState();
}

class _SplashScreenState extends State<SplashScreen> {
  final viewModel = MainViewModel(KakaoLogin());
  int? kakaoId;
  int? userId;
  int? mainHomeId;
  bool? newbie;
  Map<String, dynamic>? userinfo;

  @override
  Widget build(BuildContext context) {
    return SafeArea(
      child: Material(
        type: MaterialType.transparency,
        child: Container(
          decoration: BoxDecoration(color: Colors.blue[200]),
          child: SafeArea(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Text(
                  'LOGIN',
                  style: TextStyle(
                      fontWeight: FontWeight.w900,
                      fontSize: 40,
                      color: Colors.white),
                ),
                Padding(
                  padding: const EdgeInsets.all(16.0),
                  child: Container(
                      decoration: BoxDecoration(),
                      child: Image.asset('asset/img/logo.png')),
                ),
                // AnimatedSwitcher(
                //   transitionBuilder: wrapAnimatedBuilder,
                //   duration: Duration(milliseconds: 500),
                //   child: viewModel.isLogined == false
                //       ? Text(
                //           '로그인 해주세요',
                //           style: TextStyle(fontSize: 20, color: Colors.white),
                //         )
                //       : Text('환영합니다',
                //           style: TextStyle(fontSize: 20, color: Colors.white)),
                // ),
                Login(),
              ],
            ),
          ),
        ),
      ),
    );
  }

  Widget wrapAnimatedBuilder(Widget widget, Animation<double> animation) {
    final rotate = Tween(begin: pi, end: 0.0).animate(animation);

    return AnimatedBuilder(
      animation: rotate,
      child: widget,
      builder: (_, widget) {
        return Transform(
          transform: Matrix4.rotationY(rotate.value),
          child: widget,
          alignment: Alignment.center,
        );
      },
    );
  }

  Widget Login() {
    return Container(
        child:
            Column(mainAxisAlignment: MainAxisAlignment.spaceAround, children: [
      // Text(
      //   '${viewModel.user?.kakaoAccount?.email ?? ''} ',
      //   style: Theme.of(context).textTheme.bodySmall,
      // ),
      SizedBox(
        child: GestureDetector(
            onTap: () async {
              final HASH = await KakaoSdk.origin;
              print(HASH);
              await viewModel.login();
              setState(() {});
              if (await viewModel.isLogined) {
                kakaoId = await viewModel.user?.id;
                var result = await postLogin(viewModel.user?.id);
                userId = result['userId'];
                mainHomeId = result['mainHomeId'];
                newbie = result['new'];
                print(result);
                // postLogin() 함수가 반환한 결과(result)를 처리하는 로직
              } else {
                toast(context, '로그인 실패');
              }

              if (await viewModel.isLogined && mainHomeId != 0) {
                var Robot = await getHomeName(userId, mainHomeId);
                print('dddddd$Robot');
                Navigator.of(context).pushReplacement(MaterialPageRoute(
                    builder: (BuildContext context) => MainScreen(
                        userId: userId,
                        username:
                            viewModel.user?.kakaoAccount?.profile?.nickname,
                        kakaoID: viewModel.user?.id,
                        homeId: mainHomeId,
                        profile: viewModel
                                .user?.kakaoAccount?.profile?.profileImageUrl
                                .toString() ??
                            '',
                        homename: Robot)));
              } else if (await viewModel.isLogined && mainHomeId == 0) {
                Navigator.of(context).pushReplacement(
                  MaterialPageRoute(
                    builder: (BuildContext context) => RegisterScreen(
                        userId: userId,
                        username:
                            viewModel.user?.kakaoAccount?.profile?.nickname,
                        kakaoID: viewModel.user?.id,
                        profile: viewModel
                                .user?.kakaoAccount?.profile?.profileImageUrl
                                .toString() ??
                            ''),
                  ),
                );
              }
            },
            child: Image.asset('asset/img/kakao_login.png')),
      ),
      SizedBox(height: 16),
      viewModel.isLogined != true
          ? GestureDetector(
              onTap: () async {
                await viewModel.logout();
                setState(() {});
              },
              child: Text(
                '로그아웃 하기',
                style: TextStyle(color: Colors.grey),
              ))
          : SizedBox()
    ]));
  }
}

class MainViewModel {
  final SocialLogin _socialLogin;
  bool isLogined = false;
  User? user;

  MainViewModel(this._socialLogin);

  Future login() async {
    isLogined = await _socialLogin.login();
    print(isLogined);
    if (isLogined) {
      user = await UserApi.instance.me();
    }
  }

  Future logout() async {
    await _socialLogin.logout();
    isLogined = false;
    user = null;
  }
}

abstract class SocialLogin {
  Future<bool> login();

  Future<bool> logout();
}
