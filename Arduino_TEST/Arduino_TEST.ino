﻿/*
 Name:		Arduino_TEST.ino
 Created:	2018/12/16 13:20:56
 Author:	xbina
*/

#include <Servo.h>		// 声明调用Servo.h库
Servo myservo;			// 创建一个舵机对象
int pos = 0;			// 变量pos用来存储舵机位置
// the setup function runs once when you press reset or power the board
void setup() {
	myservo.attach(9);	// 将引脚9上的舵机与声明的舵机对象连接起来
}

// the loop function runs over and over again until power down or reset
void loop() {
	for (pos = 0; pos < 180; pos += 1) { // 舵机从0°转到180°，每次增加1°? ?? ?? ??
		myservo.write(pos);				// 给舵机写入角度? ?
		delay(15);						// 延时15ms让舵机转到指定位置
	}
	for (pos = 180; pos >= 1; pos -= 1) {// 舵机从180°转回到0°，每次减小1°
		myservo.write(pos);// 写角度到舵机? ???
		delay(15);// 延时15ms让舵机转到指定位置
	}
}