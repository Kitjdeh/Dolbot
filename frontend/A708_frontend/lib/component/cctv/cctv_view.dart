import 'dart:typed_data';

import 'package:dolbot/sockect/sockect.dart';
import 'package:flutter/material.dart';
import 'package:image_picker/image_picker.dart';
import 'dart:convert';
import 'package:socket_io_client/socket_io_client.dart' as IO;
import 'dart:async';

StreamSocket streamSocket = StreamSocket();

class CctvView extends StatefulWidget {
  const CctvView({Key? key}) : super(key: key);

  @override
  State<CctvView> createState() => _CctvViewState();
}

@override
bool get wantKeepAlive => true;

class _CctvViewState extends State<CctvView>
    with AutomaticKeepAliveClientMixin<CctvView> {
  @override
  bool get wantKeepAlive => true;

  IO.Socket? socket;
  Image? RosImage;
  final _cctvimage = StreamController<Image>();
  @override
  void dispose() {
    super.dispose();
    _cctvimage.close();
  }

  @override
  void initState() {
    print('11');
    super.initState();
    // Connect to the socket server
    IO.Socket socket = IO.io('http://3.36.67.119:8081',
        IO.OptionBuilder().setTransports(['websocket']).build());
    socket.onConnect((_) {
      // socket.emit('user_message', message);
    });
    // Listen for weather updates
    socket.on('video', (data) async {
      if (mounted) {
        var Data = jsonDecode(data);
        var ImageData = (Data['data'] as List<dynamic>).cast<int>();
        Uint8List _byte = Uint8List.fromList(ImageData);
        print('coming');
        setState(() {
          RosImage =
              Image.memory(_byte, width: 320, height: 240, fit: BoxFit.fill);
          _cctvimage.add(RosImage!);
        });
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    super.build(context);
    return StreamBuilder<Image>(
        stream: _cctvimage.stream,
        builder: (context, snapshot) {
          return Container(child: RosImage);
        });
  }
}
