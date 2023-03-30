import 'package:dolbot/component/login/kakao_login.dart';
import 'package:dolbot/restapi/log_rest_api.dart';
import 'package:dolbot/screen/home_screen.dart';
import 'package:dolbot/screen/main/main_screen.dart';
import 'package:flutter/material.dart';
import 'package:kakao_flutter_sdk/kakao_flutter_sdk_user.dart';

class SplashScreen extends StatefulWidget {
  const SplashScreen({Key? key}) : super(key: key);

  @override
  State<SplashScreen> createState() => _SplashScreenState();
}

class _SplashScreenState extends State<SplashScreen> {
  final viewModel = MainViewModel(KakaoLogin());
  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(color: Colors.blue[200]),
      child: SafeArea(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(
              'LOGIN',
              style: TextStyle(color: Colors.white),
            ),
            Container(
              child: Image.asset('asset/img/logo.png'),
            ),
            Login(),
          ],
        ),
      ),
    );
  }

  Widget Login() {
    return Container(
        child: Column(
      children: [
        SizedBox(
          width: 400,
          child: Image.network(
              viewModel.user?.kakaoAccount?.profile?.profileImageUrl ?? ''),
        ),
        Text(
          '${viewModel.isLogined}',
          style: Theme.of(context).textTheme.headline4,
        ),
        ElevatedButton(
          onPressed: () async {
            await viewModel.login();
            Navigator.of(context).pushReplacement(MaterialPageRoute(
                builder: (BuildContext context) => MainScreen()));

            setState(() {});
          },
          child: const Text('Login'),
        ),
        ElevatedButton(
          onPressed: () async {
            await viewModel.logout();
            setState(() {});
          },
          child: const Text('Logout'),
        ),
      ],
    ));
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
