#pragma once
void CW()//正转加速
{
	int i = 0;
	for (i = 0; i < 255; i++)
	{
		analogWrite(PWM1, i);
		analogWrite(PWM2, i);
		digitalWrite(IN1, HIGH);
		digitalWrite(IN2, LOW);
		digitalWrite(IN3, HIGH);
		digitalWrite(IN4, LOW);
		delay(10);
	}
}
void CCW()//反转加速
{
	int i = 0;
	for (i = 0; i < 255; i++)
	{
		analogWrite(PWM1, i);
		analogWrite(PWM2, i);
		digitalWrite(IN1, LOW);
		digitalWrite(IN2, HIGH);
		digitalWrite(IN3, LOW);
		digitalWrite(IN4, HIGH);
		delay(10);
	}
}