import 'package:flutter/material.dart';
import 'package:sleek_circular_slider/sleek_circular_slider.dart';
import 'dart:io';

class AirCondition extends StatefulWidget {
  AirCondition({Key? key}) : super(key: key);

  @override
  State<AirCondition> createState() => _AirConditionState();
}

class _AirConditionState extends State<AirCondition> {
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: Container(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Text(
                  '에어컨',
                  style: TextStyle(fontSize: 30, fontWeight: FontWeight.w700),
                ),
              ],
            ),
            Container(
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(100),
                image: new DecorationImage(
                    image: new AssetImage('asset/img/weather_snow.gif'),
                    fit: BoxFit.none),
              ),
              child: SleekCircularSlider(
                appearance: CircularSliderAppearance(
                    customWidths: CustomSliderWidths(),
                    size: MediaQuery.of(context).size.height / 10 * 3,
                    infoProperties: InfoProperties(
                        bottomLabelText: '희망온도',
                        bottomLabelStyle: TextStyle(
                          fontSize: 20,
                          fontWeight: FontWeight.w700,
                          color: Colors.white,
                        ),
                        mainLabelStyle: TextStyle(
                          fontSize: 50.0,
                          fontWeight: FontWeight.w700,
                          color: Colors.white,
                        ),
                        modifier: percentageModifier),
                    customColors: CustomSliderColors(
                        trackColor: Colors.blue[100],
                        progressBarColor: Colors.blue)),
                initialValue: InitTemp,
                onChange: (double value) {
                  setState(() {
                    InitTemp = value;
                  });
                },
                max: 50,
                min: 0,
              ),
            ),
            SizedBox(
              height: 30,
              child: Row(
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
            ),
            SizedBox(
              height: 30,
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: [
                  Text(
                    '바람세기',
                    style: TextStyle(fontWeight: FontWeight.w700),
                  ),
                  ToggleButtons(
                    textStyle: TextStyle(fontWeight: FontWeight.w700),
                    selectedBorderColor: Colors.blue,
                    borderRadius: BorderRadius.circular(100),
                    isSelected: isSelectedPower,
                    onPressed: (int index) {
                      setState(() {
                        for (int buttonIndex = 0;
                            buttonIndex < isSelectedPower.length;
                            buttonIndex++) {
                          if (buttonIndex == index) {
                            isSelectedPower[buttonIndex] = true;
                          } else {
                            isSelectedPower[buttonIndex] = false;
                          }
                        }
                      });
                    },
                    children: [Text('약풍'), Text('난방'), Text('제습')],
                  ),
                ],
              ),
            )
          ],
        ),
      ),
    );
  }
}

List<bool> isSelectedMode = [true, false, false];
List<bool> isSelectedPower = [true, false, false];
double InitTemp = 30.0;
String percentageModifier(double value) {
  final roundedValue = value.ceil().toInt().toString();
  return '$roundedValue ℃';
}

void onSliderChanged(double val) {}
