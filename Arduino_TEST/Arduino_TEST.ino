/*
 Name:		Arduino_TEST.ino
 Created:	2018/12/16 13:20:56
 Author:	xbina
*/
#define ARM_LEFT	10	//左舵机
#define ARM_RIGHT	11	//右舵机
#include <Servo.h>		// 声明调用Servo.h库
Servo myservo,a,b;
int pos = 0;			// 变量pos用来存储舵机位置
// the setup function runs once when you press reset or power the board
void setup() {
	myservo.attach(9);	// 将引脚9上的舵机与声明的舵机对象连接起来
	a.attach(ARM_LEFT);
	b.attach(ARM_RIGHT);
	Serial.begin(9600);
	b.write(0);
}
// the loop function runs over and over again until power down or reset
void loop() {
}