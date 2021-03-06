
import 'dart:convert';
import 'dart:math';
import 'dart:typed_data';

import 'package:flame/game.dart';
import 'package:flutter/material.dart';
import 'dart:io';
import 'RobotGame.dart';
void main(){
  runApp(MyApp());

}

class MyApp extends StatelessWidget {
  // This widget is the root of your application.

  @override
  Widget build(BuildContext context) {

    return MaterialApp(
      title: 'Robot',
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: MyHomePage(title: 'Robot Client',socket: Socket.connect('localhost', 5555)
      ),

    );
  }
}

class MyHomePage extends StatefulWidget {
  MyHomePage({Key key, this.title,this.socket}) : super(key: key);

  final String title;
  final Future<Socket> socket;

  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  String startX;
  String startY;
  String targetX;
  String targetY;
  Point pos;



  String color='';




  @override
  void initState() {


  }


  @override
  Widget build(BuildContext context) {


    return Scaffold(
      appBar: AppBar(
        // Here we take the value from the MyHomePage object that was created by
        // the App.build method, and use it to set our appbar title.
        title: Text(widget.title),
      ),
      body: RobotGame(widget.socket).widget,


      drawer: Drawer(
        child: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              DropdownButton<String>(
              value: color,
                icon: Icon(Icons.arrow_downward),
                iconSize: 24,
                elevation: 16,
                style: TextStyle(
                    color: Colors.deepPurple
                ),
                underline: Container(
                  height: 2,
                  color: Colors.deepPurpleAccent,
                ),
                onChanged: (String newValue) {
                  setState(() {
                    color = newValue;
                  });
                },
                items: <String>['','Red', 'Green', 'Blue', 'Yellow']
                    .map<DropdownMenuItem<String>>((String value) {
                  return DropdownMenuItem<String>(
                    value: value,
                    child: Text(value),
                  );
                })
                    .toList(),
              ),
              Padding(
                  padding: EdgeInsets.fromLTRB(25, 20, 25, 0),
                  child: TextField(
                    decoration: InputDecoration(
                        hintText: 'startX',
                        border: OutlineInputBorder()
                    ),

                    onChanged: (val){
                      startX=val;
                    },
                  )

              ),
              Padding(
                  padding: EdgeInsets.fromLTRB(25, 20, 25, 0),
                  child: TextField(
                    decoration: InputDecoration(
                        hintText: 'startY',
                        border: OutlineInputBorder()
                    ),

                    onChanged: (val){
                      startY=val;
                    },
                  )

              ),
              Padding(
                  padding: EdgeInsets.fromLTRB(25, 20, 25, 0),
                  child: TextField(
                    decoration: InputDecoration(
                        hintText: 'targetX',
                        border: OutlineInputBorder()
                    ),

                    onChanged: (val){
                      targetX=val;
                    },
                  )

              ),
              Padding(
                  padding: EdgeInsets.fromLTRB(25, 20, 25, 0),
                  child: TextField(
                    decoration: InputDecoration(
                        hintText: 'targetY',
                        border: OutlineInputBorder()
                    ),

                    onChanged: (val){
                      targetY=val;
                    },
                  )

              ),

              IconButton(
                icon: Icon(Icons.cloud_upload),
                onPressed: (){
                  widget.socket.then((socket) {
                    print("sending");

                    socket.write(jsonEncode({'startX':startX,'startY':startY,'color':color,'targetX':targetX,'targetY':targetY}));
                    socket.flush();

                  });

                },
              )

            ],
          ),
        ),
      ),


    );

  }
  void dataHandler(Uint8List data){

    var jsonData=json.decode(String.fromCharCodes(data).trim());

  }





}
