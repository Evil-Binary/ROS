#pragma once

#define FALSE 0
#define TRUE 1

void aCount_CallBack();
void bCount_CallBack();
void cCount_CallBack();

void Forward(float speed, int time);
void Back(float speed, int time);
void Left(float speed, int time);
void Right(float speed, int time);
void Y_Left(int time);
void Y_Right(int time);
void Stop();

void r_ser(Servo ser, int angle, bool Z);