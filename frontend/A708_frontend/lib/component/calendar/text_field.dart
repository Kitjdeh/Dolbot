import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:intl/intl.dart';

class CustomTextField extends StatefulWidget {
  final String label;
  final String initialValue;
  // true - 시간 / false - 내용
  final bool isTime;
  final FormFieldSetter<String> onSaved;
  const CustomTextField(
      {required this.initialValue,
      required this.onSaved,
      required this.label,
      required this.isTime,
      Key? key})
      : super(key: key);

  @override
  State<CustomTextField> createState() => _CustomTextFieldState();
}

String? _initialValue;
set initialValue(String? value) {
  _initialValue = value;
  // Do any other necessary actions when the 'initialValue' is set.
}

class _CustomTextFieldState extends State<CustomTextField> {
  String? yearMonthDayTime;
  TextEditingController ymdtController = TextEditingController();
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

        ymdtController.text = '${dateTime.toString().split(' ')[0]} $hour:$min';
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      height: 100,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            '${widget.label}',
            style: TextStyle(
              color: Colors.blue,
              fontWeight: FontWeight.w600,
            ),
          ),
          if (widget.isTime) DateTimeHourField(context),
          if (!widget.isTime) Expanded(child: renderTextField(context)),
        ],
      ),
    );
  }

  Widget renderDateField(BuildContext context) {
    String? selectedday = _initialValue;
    TextEditingController dateCtl = TextEditingController(text: selectedday);
    DateTime? date;
    return TextFormField(
      onTap: () async {
        FocusScope.of(context).requestFocus(new FocusNode());
        date = await showDatePicker(
            context: context,
            initialDate: DateTime.now(),
            firstDate: DateTime(1900),
            lastDate: DateTime(2100));
        if (date != null) {
          selectedday = DateFormat('yyyy-MM-dd').format(date!);
          widget.onSaved(selectedday);
          dateCtl.text =
              selectedday!; // update the TextEditingController's text value
          setState(() {
            _initialValue = selectedday;
          });
        }
      },
      controller: dateCtl,
      decoration: InputDecoration(
        border: InputBorder.none,
        filled: true,
        fillColor: Colors.blue[100],
      ),
    );
  }

  Widget renderTextField(BuildContext context) {
    return TextFormField(
      readOnly: widget.isTime == true ? true : false,
      onSaved: widget.onSaved,
      //null이 return 되면 에러가 없다.
      //에러가 있으면 에러를 String 값으로 리턴해준다.
      validator: (String? val) {
        if (val == null || val.isEmpty) {
          return '값을 입력해주세요';
        }
        //숫자가 안들어왔을 때
        if (val.length > 500) {
          return '500자 이하의 글자를 입력해주세요';
        }

        return null;
      },
      cursorColor: Colors.grey,
      maxLines: widget.isTime ? 1 : null,
      initialValue:
          widget.isTime ? widget.initialValue.toString() : widget.initialValue,
      expands: widget.isTime ? false : true,
      maxLength: widget.isTime ? 10 : 20,
      keyboardType:
          widget.isTime ? TextInputType.datetime : TextInputType.multiline,
      // inputFormatters: isTime ? [FilteringTextInputFormatter.] : [],
      decoration: InputDecoration(
        border: InputBorder.none,
        filled: true,
        fillColor: Colors.blue[100],
        // suffixText: isTime ? '' : null,
      ),
    );
  }

  Widget DateTimeHourField(BuildContext context) {
    return GestureDetector(
      onTap: yearMonthDayTimePicker,
      child: AbsorbPointer(
        child: TextFormField(
          controller: ymdtController,
          decoration: InputDecoration(
            border: InputBorder.none,
            filled: true,
            fillColor: Colors.blue[100],
            // suffixText: isTime ? '' : null,
          ),
          onSaved: (val) {
            yearMonthDayTime = ymdtController.text;
          },
          validator: (val) {
            if (val == null || val.isEmpty) {
              return 'Year-Month-Date-Time is necessary';
            }
            return null;
          },
        ),
      ),
    );
  }
}
