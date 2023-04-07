import 'dart:convert';

import 'package:dolbot/component/alert/toast.dart';
import 'package:dolbot/restapi/user_rest_api.dart';
import 'package:dolbot/screen/main/main_screen.dart';
import 'package:dolbot/screen/splash_screen.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:intl/intl.dart';

class RegisterScreen extends StatefulWidget {
  // int? kakaoId;
  int? userId;
  // int? mainHomeId;
  // bool? newbie;
  // Map<String, dynamic>? userinfo;
  String? username;
  int? kakaoID;
  String? profile;

  RegisterScreen(
      {this.userId, this.username, this.kakaoID, this.profile, Key? key})
      : super(key: key);
  @override
  State<RegisterScreen> createState() => _RegisterScreenState();
}

class _RegisterScreenState extends State<RegisterScreen> {
  final GlobalKey<FormState> _formKey = GlobalKey<FormState>();
  TextEditingController _nicknameController = new TextEditingController();
  TextEditingController _robotnumController = new TextEditingController();
  int? _robotnum;
  String? _nickname;
  @override
  void dispose() {
    _nicknameController.dispose();
    _robotnumController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final bottomInset = MediaQuery.of(context).viewInsets.bottom;
    // print(widget.username);
    // print(widget.kakaoID);
    // print(widget.userId);
    return Scaffold(
      body: SafeArea(
        child: GestureDetector(
          onTap: () {
            FocusScope.of(context).requestFocus(FocusNode());
          },
          child: Container(
            height: MediaQuery.of(context).size.height + bottomInset * 2,
            decoration: BoxDecoration(color: Colors.blue[200]),
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Text(
                  '기기를 등록해주세요',
                  style: TextStyle(color: Colors.white),
                ),
                SizedBox(
                  height: 80,
                  child: Container(
                      decoration: BoxDecoration(
                        borderRadius: BorderRadius.circular(150),
                      ),
                      child: Image.asset('asset/img/logo.png')),
                ),
                SizedBox(
                  child: Text('환영합니다 ${widget.username ?? ''}님',
                      style: TextStyle(fontSize: 20, color: Colors.white)),
                ),
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 32.0),
                  child: Column(
                    children: [
                      Text('닉네임을 입력하세요'),
                      TextFormField(
                        controller: _nicknameController,
                        validator: (value) {
                          if (value!.isEmpty) {
                            return '입력값이 없습니다.';
                          }
                          return null;
                        },
                        decoration: InputDecoration(
                          border: InputBorder.none,
                          filled: true,
                          fillColor: Colors.blue[100],
                          // suffixText: isTime ? '' : null,
                        ),
                        keyboardType: TextInputType.text,
                        maxLines: 1,
                      ),
                    ],
                  ),
                ),
                SizedBox(
                  height: 40 - bottomInset / 10,
                ),
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 32.0),
                  child: Column(
                    children: [
                      Text('로봇 번호를 입력해주세요'),
                      TextFormField(
                        controller: _robotnumController,
                        validator: (value) {
                          if (value!.isEmpty) {
                            return '비어있습니다.';
                          }
                          return null;
                        },
                        decoration: InputDecoration(
                          border: InputBorder.none,
                          filled: true,
                          fillColor: Colors.blue[100],
                          // suffixText: isTime ? '' : null,
                        ),
                        keyboardType: TextInputType.number,
                        maxLines: 1,
                      ),
                    ],
                  ),
                ),
                ElevatedButton(
                    onPressed: () async {
                      _nickname = await _nicknameController.text;
                      _robotnum = await int.parse(_robotnumController.text);
                      var response = await postRobotAdd(
                          widget.userId!, _nickname!, _robotnum!);
                      if (response.statusCode == 200) {
                        final decodedBody =
                            await utf8.decode(response.bodyBytes);
                        final result = await jsonDecode(decodedBody);
                        final nickname = await result['nickname'];
                        final HomeId = await result['homeId'];
                        print(result);
                        await Navigator.of(context)
                            .pushReplacement(MaterialPageRoute(
                                builder: (BuildContext context) => MainScreen(
                                      username: widget.username,
                                      kakaoID: widget.kakaoID,
                                      profile: widget.profile,
                                      homename: nickname,
                                      homeId: HomeId,
                                    )));
                      } else if (response.statusCode == 409) {
                        toast(context, '이미 가지고 있는 robotNumber입니다.');
                      } else {
                        toast(context, '존재하지 않는 robotNumber입니다');
                      }
                    },
                    child: Text('기기등록')),
                ElevatedButton(
                    onPressed: () {
                      Navigator.of(context).pushReplacement(
                        MaterialPageRoute(
                            builder: (BuildContext context) => SplashScreen()),
                      );
                    },
                    child: Text('로그인페이지로')),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
