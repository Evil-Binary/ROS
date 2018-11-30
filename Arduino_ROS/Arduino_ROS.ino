/*
 Name:		Arduino_ROS.ino
 Created:	2018/11/30 21:54:36
 Author:	xbina
*/

// the setup function runs once when you press reset or power the board

#include <Arduino.h>
#include "L298N_Motor.h"

int IN1 = 4;  //IN1　port 
int IN2 = 5;  //IN2  port
int PWM1 = 6; //EN1 port

int IN3 = 7;  //IN3 port
int IN4 = 8;  //IN4 port
int PWM2 = 9; //EN2 port

void setup() {
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(PWM1, OUTPUT); //定义PWM输出
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);
	pinMode(PWM2, OUTPUT); //定义PWM输出
}


// the loop function runs over and over again until power down or reset
void loop() {
	CW();//正转
	delay(2000);//延迟1秒
	CCW();//反转
	delay(5000);//延迟1秒
}
