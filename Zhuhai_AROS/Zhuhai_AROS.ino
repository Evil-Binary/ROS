/*
 Name:		Zhuhai_AROS.ino
 Created:	2018/12/8 2:39:19
 Author:	xbina
*/

#include <Arduino.h>
#include <Servo.h>;
#include <ros.h>;
#include <PID_v1.h>
#include "Zhuhai_AROS.h"

//电机端口
#define DIR1A 6		//Motor 1A
#define DIR1B 7		//Motor 1B
#define DIR1P 		//Motor 1 PWM

#define DIR2A 8		//Motor 2A
#define DIR2B 9		//Motor 2B
#define DIR2P 		//Motor 2 PWM

#define DIR3A 10	//Motor 3A
#define DIR3B 11	//Motor 3B
#define DIR3P 		//Motor 3 PWM

#define DIR4A 12	//Motor 4A
#define DIR4B 13	//Motor 4B
#define DIR4P		//Motor 4 PWM
//舵机端口
#define ARM_CLAW	2	//机械臂爪舵机
#define ARM_BELOW	3	//机械臂旋转舵机
#define ARM_LEFT	4	//左舵机
#define ARM_RIGHT	5	//右舵机

Servo a_claw, a_below, a_left, a_right;
int a_c_angle, a_b_angle, a_l_angle, a_r_angle = 180;

/*****************************************************************

  L298N:
  IN1	IN2		运动
  ------------------
  LOW	LOW		停止
  HIGH	LOW		正转
  LOW	HIGH	反转
  HIGH	HIGH	制动
  (制动就是短路电机线圈，达到制动效果)
  IN1的输入是HIGH/LOW
  IN2的输入是HIGH/LOW/PWM
  调速使用IN2
		Y
		*A
				X
	*B		*C
	LEFT	RIGHT

  电机减速比不允许使用'='，使用'<'/'>'代替
  The End...
*****************************************************************/

// the setup function runs once when you press reset or power the board
void setup() {
	// put your setup code here, to run once:
	//初始化串口通信，通信双方bps必须一致
	Serial.begin(9600);

	//设置舵机端口
	a_claw.attach(ARM_CLAW);
	a_below.attach(ARM_BELOW);
	a_left.attach(ARM_LEFT);
	a_right.attach(ARM_RIGHT);

	//设置控制L298N电机端口为输出
	pinMode(DIR1A, OUTPUT);
	pinMode(DIR1B, OUTPUT);
	pinMode(DIR2A, OUTPUT);
	pinMode(DIR2B, OUTPUT);
	pinMode(DIR3A, OUTPUT);
	pinMode(DIR3B, OUTPUT);
	pinMode(DIR4A, OUTPUT);
	pinMode(DIR4B, OUTPUT);

	delay(1000);	//延迟1秒
}

// the loop function runs over and over again until power down or reset
void loop()
{
	a_claw.write(a_c_angle);
	a_claw.write(a_c_angle);
	a_claw.write(a_c_angle);
	delay(100);

	Forward();
	delay(2000);
	Stop();

	Back();
	delay(2000);
	Stop();
	
	Left();
	delay(2000);
	Stop();

	Right()
	
		;
	delay(2000);
	Stop();

	Y_Left();
	delay(2000);
	Stop();

	Y_Right();
	delay(2000);
	Stop();
	/*
	if (Serial.available() > 0) {//如果缓存区有东西
		switch (Serial.read()) {
		case 'P':
			Serial.println("P,okey");
			break;

			//前
		case 'f':
			Forward();
			delay(1000);
			Stop();
			Serial.println("forward,okey");
			break;

			//后
		case 'b':
			Back();
			delay(1000);
			Stop();
			Serial.println("back,okey");
			break;

			//左
		case 'l':
			Left();
			delay(1000);
			Stop();
			Serial.println("left,okey");
			break;

			//右
		case 'r':
			Right();
			delay(1000);
			Stop();
			Serial.println("right,okey");
			break;
			//左
		case '5':
			Y_Left();
			delay(1000);
			Stop();
			Serial.println("left,okey");
			break;

			//右
		case '6':
			Y_Right();
			delay(1000);
			Stop();
			Serial.println("right,okey");
			break;

		case 's':
			Stop();
			Serial.println("stop,okey");
			break;

		case '1':
			Serial.println("1");
			break;

		case '2':
			break;

		case '3':
			break;

		case '4':
			break;

		default:
			break;
		}
	}*/
}

//前进，Y轴上下移动
void Forward()
{
	//A轮电机
	digitalWrite(DIR1A, HIGH);
	digitalWrite(DIR1B, LOW);

	//B轮电机
	digitalWrite(DIR2A, LOW);
	digitalWrite(DIR2B, HIGH);

	//C轮置0
	analogWrite(DIR3B, 0);
}

//后退，Y轴上下移动
void Back()
{
	digitalWrite(DIR1A, LOW);
	digitalWrite(DIR1B, HIGH);

	digitalWrite(DIR2A, HIGH);
	digitalWrite(DIR2B, LOW);

	analogWrite(DIR3B, 0);
}

//左平移，X轴移动
void Left()
{
	digitalWrite(DIR1A, LOW);
	digitalWrite(DIR1B, HIGH);

	digitalWrite(DIR2A, LOW);
	digitalWrite(DIR2B, HIGH);

	digitalWrite(DIR3A, HIGH);
	digitalWrite(DIR3B, LOW);
}

//右平移，X轴移动
void Right()
{
	digitalWrite(DIR1A, HIGH);
	digitalWrite(DIR1B, LOW);

	digitalWrite(DIR2A, HIGH);
	digitalWrite(DIR2B, LOW);

	digitalWrite(DIR3A, LOW);
	digitalWrite(DIR3B, HIGH);
}

//左原地转
void Y_Left()
{
	digitalWrite(DIR1A, LOW);
	digitalWrite(DIR1B, HIGH);

	digitalWrite(DIR2A, LOW);
	digitalWrite(DIR2B, HIGH);

	digitalWrite(DIR3A, LOW);
	digitalWrite(DIR3B, HIGH);
}

//右原地转
void Y_Right()
{

	digitalWrite(DIR1A, HIGH);
	digitalWrite(DIR1B, LOW);

	digitalWrite(DIR2A, HIGH);
	digitalWrite(DIR2B, LOW);

	digitalWrite(DIR3A, HIGH);
	digitalWrite(DIR3B, LOW);
}

//停止
void Stop()
{
	digitalWrite(DIR1A, LOW);
	digitalWrite(DIR1B, LOW);
	digitalWrite(DIR2A, LOW);
	digitalWrite(DIR2B, LOW);
	digitalWrite(DIR3A, LOW);
	digitalWrite(DIR3B, LOW);
}