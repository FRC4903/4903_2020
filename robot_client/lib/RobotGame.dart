

import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';
import 'dart:ui';

import 'package:flame/components/component.dart';
import 'package:flame/game/game.dart';
import 'package:flame/gestures.dart';
import 'package:flame/position.dart';
import 'package:flame/sprite.dart';
import 'package:flame/text_config.dart';
import 'package:flutter/widgets.dart';

class RobotGame extends BaseGame with TapDetector,HasWidgetsOverlay{
  static bool listened=false;
  SpriteComponent robot;
  String widgetName="Pos";
  List<Offset> path=List();
  Offset target=Offset(0,0);
  final TextConfig config = TextConfig(fontSize: 14, fontFamily: 'Awesome Font',color: Color.fromRGBO(255, 255, 255, 1));

  RobotGame(Future<Socket> socket) {



    socket.then((s){
      if(listened) return;
      else listened=true;
      s.listen(dataHandler,
          onError: errorHandler,
          onDone: (){
        s.destroy();
        exit(0);
        },
          cancelOnError: false);
    });


    robot=SpriteComponent.fromSprite(100, 100, Sprite('Robot.png'));
    robot.x=300;
    robot.y=300;


  }

  @override
  void update(double t) {

  }

  @override
  void render(Canvas canvas) {
    robot.render(canvas);
    config.render(canvas, "${target.dx.toInt()} ${target.dy.toInt()}", Position(200, 0));
    Offset prev;
    Paint paint=Paint();
    paint.color=Color.fromRGBO(255, 0, 0, 1);
    for (var p in path){
      if(prev!=null){
        canvas.drawLine(prev, p, paint);

      }
      prev=p;

    }
  }
  void dataHandler(Uint8List data){

    var jsonData=json.decode(String.fromCharCodes(data).trim());

    robot.x=jsonData['deltaX'];
    robot.y=jsonData['deltaY'];
  }
  void errorHandler(error, StackTrace trace){
    print(error);
  }

  @override
  void onTapDown(TapDownDetails details) {
    target=details.globalPosition;
    path.add(target);

  }


}