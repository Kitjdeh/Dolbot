import 'package:dolbot/restapi/user_rest_api.dart';
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
    return Scaffold(
      body: SafeArea(
        child: GestureDetector(
          onTap: () {
            FocusScope.of(context).requestFocus(FocusNode());
          },
          child: Container(
            height: MediaQuery.of(context).size.height + bottomInset,
            decoration: BoxDecoration(color: Colors.blue[200]),
            child: Padding(
              padding: EdgeInsets.only(
                  bottom: MediaQuery.of(context).viewInsets.bottom),
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Text(
                    '기기 등록',
                    style: TextStyle(color: Colors.white),
                  ),
                  Container(child: Image.asset('asset/img/logo.png')),
                  SizedBox(
                    child: Text('환영합니다 ${widget.username ?? ''}님',
                        style: TextStyle(fontSize: 30, color: Colors.white)),
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
                    height: 50,
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
                        var result = await postRobotAdd(
                            widget.userId!, _nickname!, _robotnum!);
                        int HomeId = result['homId'];
                      },
                      child: Text('기기등록'))
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }
}
