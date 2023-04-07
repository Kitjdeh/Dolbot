import 'package:dolbot/component/calendar/text_field.dart';
import 'package:dolbot/restapi/calendar_rest_api.dart';
import 'package:flutter/material.dart';
import 'package:get_it/get_it.dart';
import 'package:flutter/material.dart';
import 'package:intl/intl.dart';

class SchedulBottomSheet extends StatefulWidget {
  final DateTime selectedDate;
  final int? scheduleId;
  final int? homeId;
  final int? scheduleTime;
  final String? content;
  final String? startDate;
  final String? endDate;
  const SchedulBottomSheet(
      {required this.selectedDate,
      this.scheduleId,
      required this.homeId,
      required this.scheduleTime,
      required this.content,
      required this.startDate,
      required this.endDate,
      Key? key})
      : super(key: key);

  @override
  State<SchedulBottomSheet> createState() => _SchedulBottomSheetState();
}

class _SchedulBottomSheetState extends State<SchedulBottomSheet> {
  String? yearMonthDayTime;
  String? content;
  DateTime? selectedday;
  DateTime? SelectedTime;
  final GlobalKey<FormState> formKey = GlobalKey();
  GlobalKey<FormState> _fKey = GlobalKey<FormState>();

  TextEditingController ymdtController = TextEditingController();
  bool autovalidate = false;

  yearMonthDayTimePicker() async {
    final year = DateTime.now().year;
    String hour, min;

    final DateTime? dateTime = await showDatePicker(
      context: context,
      initialDate: DateTime.now(),
      firstDate: DateTime(year),
      lastDate: DateTime(year + 10),
    );

    if (dateTime != null) {
      final TimeOfDay? pickedTime = await showTimePicker(
        context: context,
        initialTime: TimeOfDay(hour: 0, minute: 0),
      );
      if (pickedTime != null) {
        if (pickedTime.hour < 10) {
          hour = '0' + pickedTime.hour.toString();
        } else {
          hour = pickedTime.hour.toString();
        }

        if (pickedTime.minute < 10) {
          min = '0' + pickedTime.minute.toString();
        } else {
          min = pickedTime.minute.toString();
        }

        setState(() {
          selectedday = dateTime;
          SelectedTime = DateTime(selectedday!.year, selectedday!.month,
              selectedday!.day, pickedTime.hour, pickedTime.minute);
        });

        // print('${selectedday.runtimeType}');
        print('${SelectedTime!.hour}');
        print(SelectedTime!.minute);
        ymdtController.text = '${dateTime.toString().split(' ')[0]} $hour:$min';
        print('${ymdtController.text}');
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final bottomInset = MediaQuery.of(context).viewInsets.bottom;
    return GestureDetector(
        onTap: () {
          FocusScope.of(context).requestFocus(FocusNode());
        },
        child: SafeArea(
          child: Container(
            color: Colors.white,
            height: MediaQuery.of(context).size.height / 2 + bottomInset,
            child: Padding(
              padding: EdgeInsets.only(bottom: bottomInset),
              child: Padding(
                padding: EdgeInsets.only(left: 8.0, right: 8.0, top: 16.0),
                child: Form(
                  key: formKey,
                  // autovalidateMode: AutovalidateMode.always,
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        '시간을 입력하세요',
                        style: TextStyle(
                          color: Colors.blue,
                          fontWeight: FontWeight.w600,
                        ),
                      ),
                      SizedBox(
                        height: 80,
                        child: GestureDetector(
                          onTap: yearMonthDayTimePicker,
                          child: AbsorbPointer(
                            child: TextFormField(
                              autovalidateMode: AutovalidateMode.always,
                              controller: ymdtController,
                              decoration: InputDecoration(
                                border: InputBorder.none,
                                filled: true,
                                fillColor: Colors.blue[100],
                                // suffixText: isTime ? '' : null,
                              ),
                              onSaved: (val) {
                                setState(() {
                                  yearMonthDayTime = ymdtController.text;
                                });
                              },
                              validator: (val) {
                                if (val == null || val.isEmpty) {
                                  return 'Year-Month-Date-Time is necessary';
                                }
                                return null;
                              },
                            ),
                          ),
                        ),
                      ),
                      SizedBox(height: 16.0),
                      _content(
                        onSaved: (String? val) {
                          setState(() {
                            content = val;
                          });
                        },
                        initialValue: '',
                      ),
                      // SizedBox(height: 16.0),
                      SizedBox(height: 8.0),
                      _SaveButton(
                        onPressed: onSavePressed,
                      ),
                    ],
                  ),
                ),
              ),
            ),
          ),
        ));
  }

  void onSavePressed() async {
    // formKey는 생성을 했는데
    // Form 위젯과 결합을 안했을 때 (여기선 일어나지 않음)
    if (formKey.currentState == null) {
      return;
    }
    //모든 state를 다  동시에 validate  진행한다.
    if (formKey.currentState!.validate()) {
      formKey.currentState!.save();

      DateTime outputDateTime2 = DateTime(
        SelectedTime!.year,
        SelectedTime!.month,
        SelectedTime!.day,
        SelectedTime!.hour, // Hour in 24-hour format
        SelectedTime!.minute, // Minute
        0, // Second
        0, // Millisecond
      );
      // print('selectedTime${SelectedTime!} / ${SelectedTime.runtimeType}');
      // print('yearMonthDayTime${yearMonthDayTime.runtimeType}');
      // print('아웃풋데이트타임2${outputDateTime2.toString()}');

      String formattedDateTime = outputDateTime2.toIso8601String();
      // formattedDateTime =
      //     formattedDateTime.replaceFirst(RegExp(r'Z$'), '+09:00');
      formattedDateTime = formattedDateTime.toString() + '+09:00';
      print('포메티트데이트타임$formattedDateTime');
      DateFormat outputDateFormat = DateFormat("yyyy-MM-dd");
      String startdate = outputDateFormat.format(SelectedTime!);
      String enddate = startdate;
      String scheduleTime = formattedDateTime;
      String Content = content!;
      int homeId = 1;
      var data = {
        "startDate": startdate,
        "endDate": enddate,
        "scheduleTime": scheduleTime,
        "content": Content,
        "homeId": homeId
      };
      // print(data);
      // print(data);
      postSchedule(data);
      Navigator.of(context).pop();
    }

    // 하나라도 에러가 있으면
    else {
      print('에러가 있습니다.');
    }
  }
}

class _Time extends StatelessWidget {
  final FormFieldSetter<String> onStartSaved;
  final FormFieldSetter<String> onEndSaved;
  final DateTime startInitialValue;
  final DateTime endInitialValue;
  const _Time(
      {required this.endInitialValue,
      required this.startInitialValue,
      required this.onEndSaved,
      required this.onStartSaved,
      Key? key})
      : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Expanded(
            child: CustomTextField(
          onSaved: onStartSaved,
          initialValue: DateFormat('yyyy.MM.dd').format(startInitialValue),
          label: '시작일',
          isTime: true,
        )),
        SizedBox(
          width: 16.0,
        ),
        Expanded(
            child: CustomTextField(
          initialValue: DateFormat('yyyy.MM.dd').format(endInitialValue),
          onSaved: onEndSaved,
          label: '종료일',
          isTime: true,
        ))
      ],
    );
  }
}

class _content extends StatelessWidget {
  final FormFieldSetter<String> onSaved;
  final String initialValue;
  const _content({required this.initialValue, required this.onSaved, Key? key})
      : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Expanded(
      child: CustomTextField(
        initialValue: initialValue,
        onSaved: onSaved,
        label: '내용',
        isTime: false,
      ),
    );
  }
}

class _SaveButton extends StatelessWidget {
  final VoidCallback onPressed;
  const _SaveButton({required this.onPressed, Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Expanded(
            child: ElevatedButton(
                onPressed: onPressed,
                style: ElevatedButton.styleFrom(primary: Colors.blue),
                child: Text('저장'))),
      ],
    );
  }
}
