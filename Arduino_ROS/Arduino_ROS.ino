/*
 Name:		Arduino_ROS.ino
 Created:	2018/12/1 0:22:18
 Author:	xbina
*/

#include <Arduino.h>
#include <Servo.h>;
#include <ros.h>;
#include <PID_v1.h>
#include <time.h>
#include "Arduino_ROS.h"

/**********
W前进
S后进
A左转
D右进
Q左转
E右转
Z机械臂左转
X机械臂右转
R机械手抓
F机械臂放
**********/

#define SHOW_MODE

//#define GAME_MODE

#ifdef SHOW_MODE
#ifdef GAME_MODE
#error
#endif
#endif

//电机端口
#define DIR1A 22		//Motor 1A
#define DIR1B 7		//Motor 1B /PWM

#define DIR2A 23		//Motor 2A
#define DIR2B 6		//Motor 2B / PWM

#define DIR3A 24	//Motor 3A
#define DIR3B 5	//Motor 3B / PWM

#define DIR4A 25	//Motor 4A
#define DIR4B 4	//Motor 4B / PWM

//舵机端口
#define ARM_CLAW	8	//机械臂爪舵机
#define ARM_BELOW	9	//机械臂旋转舵机
#define ARM_LEFT	10	//左舵机
#define ARM_RIGHT	11	//右舵机

//编码器中断端口
#define A_ENCODE_PIN_1 2
#define A_ENCODE_PIN_2 3
#define B_ENCODE_PIN_1 19
#define B_ENCODE_PIN_2 18
#define C_ENCODE_PIN_1 20
#define C_ENCODE_PIN_2 21

//电机参数
#define MOTOR_SPEED_REDUCTION_RATIO 74.8	//减速比
#define MOTOR_CPR 100	//编码器转一圈数值

//BPS
#define SERIAL_BPS 9600

float Vx, Vy, V1, V2, V3 = 0;

Servo a_claw;		//机械爪
Servo a_below;		//机械臂旋转
Servo a_left;		//左伺服器
Servo a_right;		//右伺服器

int a_c_angle = 90;	//机械爪伺服器角度
int a_b_angle = 90;	//机械臂旋转伺服器角度
int a_l_angle = 90;	//机械臂左伺服器角度
int a_r_angle = 90;	//机械臂右伺服器角度

int aCounter, bCounter, cCounter = 0;
int a_E_speed, b_E_speed, c_E_speed = 0;

char tmp;

/*****************************************************************

  float Vx,Vy;     //定义正交分解之后的速度
  float Speed1,Speed2,Speed3;  //定义各个轮子的速度

  Vx = speed * cos(β);  //计算X方向速度  前进、后退时β= 270°，Vx = 0，Vy = -speed
  Vy = speed * sin(β);  //计算Y方向速度   左右移动时β= 0°，Vx = speed，Vy = 0

  L是底盘中心到轮子的距离，theta是偏航角度,前进，后退，左右平移时为0,ω为角速度，值为0
  V1 = -sin(30 + theta) * Vx - cos(30 + theta) * Vy + L * ω;
  V2 = -sin(30 + theta) * Vx + cos(30 + theta) * Vy + L * ω;
  V3 = cos(theta) * Vx - sin(theta) * Vy + L * ω;

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
	*A		*B
				X
		*C
	LEFT	RIGHT

  电机减速比不允许使用'='，使用'<'/'>'代替
  The End...
*****************************************************************/

// the setup function runs once when you press reset or power the board
void setup() {
	// put your setup code here, to run once:
	//初始化串口通信，通信双方bps必须一致
	Serial.begin(SERIAL_BPS);

	//设置舵机端口
	a_claw.attach(ARM_CLAW);
	a_claw.write(a_c_angle);

	a_below.attach(ARM_BELOW);
	a_below.write(a_b_angle);
	
	a_left.attach(ARM_LEFT);
	a_left.write(a_l_angle);
	
	a_right.attach(ARM_RIGHT);
	a_right.write(a_r_angle);

	//设置控制L298N电机端口为输出
	pinMode(DIR1A, OUTPUT);
	pinMode(DIR1B, OUTPUT);
	pinMode(DIR2A, OUTPUT);
	pinMode(DIR2B, OUTPUT);
	pinMode(DIR3A, OUTPUT);
	pinMode(DIR3B, OUTPUT);
	pinMode(DIR4A, OUTPUT);
	pinMode(DIR4B, OUTPUT);

	//中断初始化
	//设置中断端口为输入
	pinMode(A_ENCODE_PIN_1, INPUT);
	pinMode(A_ENCODE_PIN_2, INPUT);
	pinMode(B_ENCODE_PIN_1, INPUT);
	pinMode(B_ENCODE_PIN_2, INPUT);
	pinMode(C_ENCODE_PIN_1, INPUT);
	pinMode(C_ENCODE_PIN_2, INPUT);

	digitalWrite(DIR4A, LOW);
	digitalWrite(DIR4B, HIGH);
	analogWrite(DIR4B, 255);

	Serial.println("init ok");
#ifdef SHOW_MODE
	//放开
	a_claw.write(90);
	//右转机械臂
	a_below.write(45);
	//伸远
	a_left.write(120);
	//抓住
	a_claw.write(0);

	Forward(130, 3000);
	Left(130, 3000);
	Right(130, 3000);
	Y_Left(200);
	Y_Right(200);
	Back(150, 1000);
	Forward(150, 1000);

	//放开
	a_claw.write(90);
#endif // SHOW_MODE

}

// the loop function runs over and over again until power down or reset
#ifdef GAME_MODE
void loop()
{
	//如果缓存区有东西
	if (Serial.available()) {
		tmp = Serial.read();
		if ('p' == tmp) {
			Serial.println("P,okey");
		}

		//前
		if ('w' == tmp) {
			Serial.println("forward,rev");
			Forward(200, 1000);
			Serial.println("forward,okey");
		}

		//后
		if ('s' == tmp) {
			Back(200, 1000);
			Serial.println("back,okey");
		}

		//左
		if ('a' == tmp) {
			Left(200, 1000);
			Serial.println("left,okey");
		}

		//右
		if ('d' == tmp) {
			Right(200, 1000);
			Serial.println("right,okey");
		}

		//原地左转
		if ('q' == tmp) {
			Y_Left(1000);
			Serial.println("y_left,okey");
		}

		//原地右转
		if ('e' == tmp) {
			Y_Right(1000);
			Serial.println("y_right,okey");
		}
		
		//抓住
		if ('r' == tmp) {
			a_claw.write(0);
			Serial.println("clawing");
		}

		//放开
		if ('f' == tmp) {
			a_claw.write(90);
			Serial.println("unclawing");
		}

		//机械臂左转
		if ('z' == tmp) {
			a_b_angle += 10;
			if (a_b_angle > 140) {
				Serial.println("WARNING!");
				a_b_angle = 140;
			}
			a_below.write(a_b_angle);
			Serial.println("Left,Arm");
		}

		//机械臂右转
		if ('x' == tmp) {
			a_b_angle -= 10;
			if (a_b_angle < 10) {
				Serial.println("WARNING!");
				a_b_angle = 10;
			}
			a_below.write(a_b_angle);
			Serial.println("Right,Arm");
		}

		//伸远
		if ('t' == tmp) {
			a_l_angle += 10;
			if (a_l_angle > 140) {
				Serial.println("WARNING!");
				a_l_angle = 140;
			}
			a_left.write(a_l_angle);
			a_left.write(a_l_angle);
			Serial.println("Long");
		}
		
		//伸近
		if ('g' == tmp) {
			a_l_angle -= 10;
			if (a_l_angle < 50) {
				Serial.println("WARNING!");
				a_l_angle = 50;
			}
			a_left.write(a_l_angle);
			a_left.write(a_l_angle);
			Serial.println("Unlong");
		}

		//高
		if ('c' == tmp) {
			a_r_angle -= 50;
			if (a_r_angle < 50) {
				Serial.println("WARNING!");
				a_r_angle = 50;
			}
			a_right.write(a_r_angle);
			a_right.write(a_r_angle);
			Serial.println("High");
			Serial.println(a_r_angle);
		}

		//低
		if ('v' == tmp) {
			a_r_angle += 10;
			if (a_r_angle > 130) {
				Serial.println("WARNING!");
				a_r_angle = 130;
			}
			a_right.write(a_r_angle);
			a_right.write(a_r_angle);
			Serial.println("Low");
			Serial.println(a_r_angle);
		}
	}
}
#endif

#ifdef SHOW_MODE
void loop(){}
#endif // SHOW_MODE


//A轮编码器中断服务函数
void aCount_CallBack()
{
	aCounter++;
}

//B轮编码器中断服务函数
void bCount_CallBack()
{
	bCounter++;
}

//C轮编码器中断服务函数 
void cCount_CallBack()
{
	cCounter++;
}

//前进，Y轴上下移动
void Forward(float speed,int time)
{
	//设置中断端口
//	attachInterrupt(A_ENCODE, aCount_CallBack, CHANGE);
//	attachInterrupt(B_ENCODE, bCount_CallBack, CHANGE);
	
	Vy = -speed;
	V1 = -sqrt(3) / 2 * Vy;
	V2 = sqrt(3) / 2 * Vy;
	
	//A轮电机
	digitalWrite(DIR1A, HIGH);
	digitalWrite(DIR1B, LOW);
//	analogWrite(DIR1B, V1);

	//B轮电机
	digitalWrite(DIR2A, LOW);
	digitalWrite(DIR2B, HIGH);
//	analogWrite(DIR2B, -V2);

	//C轮置0
	analogWrite(DIR3B, 0);

	delay(time);
	Stop();
}

//后退，Y轴上下移动
void Back(float speed, int time)
{
	Vy = -speed;
	V1 = -sqrt(3) / 2 * Vy;
	V2 = sqrt(3) / 2 * Vy;

	digitalWrite(DIR1A, LOW);
	digitalWrite(DIR1B, HIGH);
//	analogWrite(DIR1B, V1);

	digitalWrite(DIR2A, HIGH);
	digitalWrite(DIR2B, LOW);
//	analogWrite(DIR2B, -V2);

	analogWrite(DIR3B, 0);

	delay(time);
	Stop();
}

//右平移，X轴移动
void Right(float speed, int time)
{
	Vx = speed;
	V1 = -0.5 * Vx;
	V2 = -0.5 * Vx;
	V3 = Vx;

	digitalWrite(DIR1A, HIGH);
	digitalWrite(DIR1B, LOW);
//	analogWrite(DIR1B, -V1);

	digitalWrite(DIR2A, HIGH);
	digitalWrite(DIR2B, LOW);
//	analogWrite(DIR2B, -V2);

	digitalWrite(DIR3A, LOW);
	digitalWrite(DIR3B, HIGH);
//	analogWrite(DIR3B, V3);

	delay(time);
	Stop();
}

//左平移，X轴移动
void Left(float speed, int time)
{
	Vx = speed;
	V1 = -0.5 * Vx;
	V2 = -0.5 * Vx;
	V3 = Vx;

	digitalWrite(DIR1A, LOW);
	digitalWrite(DIR1B, HIGH);
//	analogWrite(DIR1B, -V1);

	digitalWrite(DIR2A, LOW);
	digitalWrite(DIR2B, HIGH);
//	analogWrite(DIR2B, -V2);

	digitalWrite(DIR3A, HIGH);
	digitalWrite(DIR3B, LOW);
//	analogWrite(DIR3B, V3);

	delay(time);
	Stop();
}

//左原地转
void Y_Left(int time)
{
	digitalWrite(DIR1A, LOW);
	digitalWrite(DIR1B, HIGH);

	digitalWrite(DIR2A, LOW);
	digitalWrite(DIR2B, HIGH);

	digitalWrite(DIR3A, LOW);
	digitalWrite(DIR3B, HIGH);

	delay(time);
	Stop();
}

//右原地转
void Y_Right(int time)
{

	digitalWrite(DIR1A, HIGH);
	digitalWrite(DIR1B, LOW);

	digitalWrite(DIR2A, HIGH);
	digitalWrite(DIR2B, LOW);

	digitalWrite(DIR3A, HIGH);
	digitalWrite(DIR3B, LOW);

	delay(time);
	Stop();
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

//舵机控制,（舵机，角度，正转/反转）
void r_ser(Servo ser, int angle, bool Z)
{
	int a;
	if (Z)
	{
		for (a = angle; a < 180; a += 1)
		{
			ser.write(a);
			delay(15);
		}
	}
	else
	{
		if (!Z)
		{
			for (a = angle; a >= 1; a -= 1) {
				ser.write(a);
				delay(15);
			}
		}
	}
}
