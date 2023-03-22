import 'package:flutter/material.dart';

class AirCleaner extends StatefulWidget {
  const AirCleaner({Key? key}) : super(key: key);

  @override
  State<AirCleaner> createState() => _AirCleanerState();
}

class _AirCleanerState extends State<AirCleaner> {
  @override
  Widget build(BuildContext context) {
    return Container(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        crossAxisAlignment: CrossAxisAlignment.center,
        children: [
          Text(
            '공기정청기',
            style: TextStyle(fontSize: 30, fontWeight: FontWeight.w700),
          ),
          Container(
            width: 250,
            height: 250,
            decoration: BoxDecoration(
                border: Border.all(color: Colors.greenAccent[100]!, width: 10.0),
                color: Colors.greenAccent,
                borderRadius: BorderRadius.circular(250.0)),
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Text('상태',
                    style:
                        TextStyle(fontSize: 20, fontWeight: FontWeight.w700)),
                Container(
                    width: MediaQuery.of(context).size.height / 5,
                    decoration: BoxDecoration(

                        borderRadius: BorderRadius.circular(100)),
                    child: Image.asset('asset/img/smile.gif')),
                Text(
                  '좋아요',
                  style: TextStyle(fontSize: 30, fontWeight: FontWeight.w700),
                )
              ],
            ),
          ),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              Text(
                '모드선택',
                style: TextStyle(fontWeight: FontWeight.w700),
              ),
              ToggleButtons(
                textStyle: TextStyle(fontWeight: FontWeight.w700),
                selectedBorderColor: Colors.blue,
                borderRadius: BorderRadius.circular(100),
                isSelected: isSelectedMode,
                onPressed: (int index) {
                  setState(() {
                    for (int buttonIndex = 0;
                        buttonIndex < isSelectedMode.length;
                        buttonIndex++) {
                      if (buttonIndex == index) {
                        isSelectedMode[buttonIndex] = true;
                      } else {
                        isSelectedMode[buttonIndex] = false;
                      }
                    }
                  });
                },
                children: [Text('약풍'), Text('난방'), Text('제습')],
              ),
            ],
          ),
        ],
      ),
    );
  }
}

List<bool> isSelectedMode = [true, false, false];
