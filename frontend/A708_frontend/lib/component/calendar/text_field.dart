import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

class CustomTextField extends StatelessWidget {
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
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          '$label',
          style: TextStyle(
            color: Colors.blue,
            fontWeight: FontWeight.w600,
          ),
        ),
        if (isTime) renderTextField(),
        if (!isTime) Expanded(child: renderTextField()),
      ],
    );
  }

  Widget renderTextField() {
    return TextFormField(
      onSaved: onSaved,
      //null이 return 되면 에러가 없다.
      //에러가 있으면 에러를 String 값으로 리턴해준다.
      validator: (String? val) {
        if (val == null || val.isEmpty) {
          return '값을 입력해주세요';
        }
        //숫자가 안들어왔을 때
        if (isTime) {
          int time = int.parse(val!);
          if (time < 0) {
            return '0 이상의 숫자를 입력해주세요';
          }
          if (time > 24) {
            return '24이하의 숫자를 입력해주세요';
          }
        } else {
          if (val.length > 500) {
            return '500자 이하의 글자를 입력해주세요';
          }
        }
        return null;
      },
      cursorColor: Colors.grey,
      maxLines: isTime ? 1 : null,
      initialValue: initialValue,
      expands: isTime ? false : true,
      maxLength: 50,
      keyboardType: isTime ? TextInputType.number : TextInputType.multiline,
      inputFormatters: isTime ? [FilteringTextInputFormatter.digitsOnly] : [],
      decoration: InputDecoration(
        border: InputBorder.none,
        filled: true,
        fillColor: Colors.blue[100],
        suffixText: isTime ? '시' : null,
      ),
    );
  }
}
