import 'package:flutter/material.dart';
import 'package:path/path.dart';

class CctvManualView extends StatelessWidget {
  const CctvManualView({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    List<int> roomNumber = [0, 1, 2, 3];

    return Container(
      decoration: BoxDecoration(color: Colors.red),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        crossAxisAlignment: CrossAxisAlignment.center,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              RoomButton(
                RoomNumber: 0,
              ),
              RoomButton(
                RoomNumber: 1,
              )
            ],
          ),
          SizedBox(
            height: 10,
          ),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              RoomButton(
                RoomNumber: 2,
              ),
              RoomButton(
                RoomNumber: 3,
              )
            ],
          ),
        ],
      ),
    );
  }
}

class RoomButton extends StatefulWidget {
  final int RoomNumber;
  const RoomButton({
    required this.RoomNumber,
  });

  @override
  State<RoomButton> createState() => _RoomButtonState();
}

class _RoomButtonState extends State<RoomButton> {
  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: 100,
      child: ElevatedButton(
          onPressed: () {
            setState(() {
              for (var i = 0; i < 4; i++) {
                i == widget.RoomNumber
                    ? selectedroom[i] = !selectedroom[i]
                    : selectedroom[i] = false;
              }
            });
          },
          style: ElevatedButton.styleFrom(
              backgroundColor: selectedroom[widget.RoomNumber] == true
                  ? Colors.blue[200]
                  : Colors.grey),
          child: Text(
            '${roomName[widget.RoomNumber]}',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.w700),
          )),
    );
  }
}

List<bool> selectedroom = [false, false, false, false];
List<String> roomName = ['안방', '거실', '화장실', '방1'];
int? selectednumber;
