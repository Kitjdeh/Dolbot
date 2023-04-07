import 'dart:typed_data';
import 'package:dolbot/sockect/sockect.dart';
import 'package:flutter/material.dart';
import 'package:image_picker/image_picker.dart';
import 'dart:convert';
import 'package:socket_io_client/socket_io_client.dart' as IO;
import 'dart:async';

StreamSocket streamSocket = StreamSocket();

class CctvView extends StatefulWidget {
  Image? RosImage;
  CctvView({this.RosImage, Key? key}) : super(key: key);

  @override
  State<CctvView> createState() => _CctvViewState();
}

@override
bool get wantKeepAlive => true;

class _CctvViewState extends State<CctvView>
    with AutomaticKeepAliveClientMixin<CctvView> {
  @override
  bool get wantKeepAlive => true;

  // IO.Socket? socket;

  Image? BeforeImage;
  StreamController<Image> _cctvimage = StreamController<Image>();
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
    // IO.Socket socket = IO.io('http://j8a708.p.ssafy.io:8081',
    //     IO.OptionBuilder().setTransports(['websocket']).build());
    // socket.onConnect((_) {
  }

  @override
  Widget build(BuildContext context) {
    if (widget.RosImage != null) {
      _cctvimage.add(widget.RosImage!);
    }
    super.build(context);
    return StreamBuilder<Image>(
        stream: _cctvimage.stream,
        builder: (context, snapshot) {
          if (snapshot.hasData) {
            return Container(child: snapshot.data);
          } else {
            return Container();
          }
        });
  }
}
