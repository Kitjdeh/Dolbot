import 'package:dolbot/component/cctv/cctv_auto_view.dart';
import 'package:dolbot/component/cctv/cctv_manual_view.dart';
import 'package:dolbot/component/cctv/cctv_view.dart';
import 'package:flutter/material.dart';
import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:flutter_toggle_tab/flutter_toggle_tab.dart';

class CctvScreen extends StatefulWidget {
  const CctvScreen({Key? key}) : super(key: key);

  @override
  State<CctvScreen> createState() => _CctvScreenState();
}

class _CctvScreenState extends State<CctvScreen> {
  bool autoselected = true;
  List<String> _Selected = ['자동모드', '수동모드'];
  int _selectedIndex = 0;
  @override
  Widget build(BuildContext context) {
    setState(() {
      getData();
    });
    return Container(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
        children: [
          Text('CCTV'),
          SizedBox(height: 300,child: CctvView()),
          FlutterToggleTab(
              width: 50,
              height: 30,
              labels: _Selected,
              selectedLabelIndex: (index) {
                setState(() {
                  _selectedIndex = index;
                });
              },
              selectedTextStyle:
                  TextStyle(color: Colors.white, fontWeight: FontWeight.w700),
              unSelectedTextStyle:
                  TextStyle(color: Colors.black87, fontWeight: FontWeight.w300),
              selectedIndex: _selectedIndex),
         SizedBox(height: 120,
             child: _selectedIndex == 0 ? CctvAutoView() : CctvManualView()),
        ],
      ),
    );
  }
}

void getData() async {
  try {
    String data = await CoinData().getCoinData();
    print(data);
    print('성공');
  } catch (e) {
    print(e);
    print('굳');
  }
}

const coinAPIURL = 'http://43.201.72.219:8080/api/v1/schedule-info/hello';
const apiKey = '시크릿';

class CoinData {
  //TODO: Create your getCoinData() method here.
  Future getCoinData() async {
    http.Response response = await http.get(Uri.parse(coinAPIURL));
    if (response.statusCode == 200) {
      var decodedData = jsonDecode(response.body);

      print(response);
      print('성공');
      return decodedData;
    } else {
      print('------------');
      print(response.statusCode);
      throw 'getCoinData() error';
    }
  }
}
