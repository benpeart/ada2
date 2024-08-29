/*
 * Motors.cpp
 *
 *  Created on: 25.09.2017
 *      Author: anonymous
 */

#include <Arduino.h>
#include "defines.h"
#include "globals.h"
#include "Motors.h"

hw_timer_t * timer1 = NULL;
hw_timer_t * timer2 = NULL;

extern "C"
{

	portMUX_TYPE muxer1 = portMUX_INITIALIZER_UNLOCKED;
	portMUX_TYPE muxer2 = portMUX_INITIALIZER_UNLOCKED;

	void IRAM_ATTR timer1ISR()
	{
		portENTER_CRITICAL_ISR(&muxer1);

		if (dir_M1 != 0)
		{
			// We generate 1us STEP pulse
			digitalWrite(PIN_MOTOR1_STEP, HIGH);
			delayMicroseconds(1);

			if (dir_M1 > 0)
				steps1--;
			else
				steps1++;

			digitalWrite(PIN_MOTOR1_STEP, LOW);
		}

		portEXIT_CRITICAL_ISR(&muxer1);
	}
	void IRAM_ATTR timer2ISR()
	{
		portENTER_CRITICAL_ISR(&muxer2);

		if (dir_M2 != 0)
		{
			// We generate 1us STEP pulse
			digitalWrite(PIN_MOTOR2_STEP, HIGH);
			delayMicroseconds(1);

			if (dir_M2 > 0)
				steps2--;
			else
				steps2++;

			digitalWrite(PIN_MOTOR2_STEP, LOW);
		}
		portEXIT_CRITICAL_ISR(&muxer2);
	}
}


void Motors_setup()
{
  // setup the timers that drive the pulses of the motors
	timer1 = timerBegin(0, 40, true);
	timerAttachInterrupt(timer1, &timer1ISR, true);
	timerAlarmWrite(timer1, ZERO_SPEED, true);

	timer2 = timerBegin(1, 40, true);
	timerAttachInterrupt(timer2, &timer2ISR, true);
	timerAlarmWrite(timer2, ZERO_SPEED, true);

	timerAlarmEnable(timer1);
	timerAlarmEnable(timer2);

	// setup the stepper motors
	pinMode(PIN_MOTORS_ENABLE, OUTPUT);
	digitalWrite(PIN_MOTORS_ENABLE, HIGH);

	// set micro stepping
#if MICROSTEPPING == 16
	pinMode(PIN_MOTOR1_MS1, OUTPUT);
	pinMode(PIN_MOTOR1_MS2, OUTPUT);
	digitalWrite(PIN_MOTOR1_MS1, HIGH);
	digitalWrite(PIN_MOTOR1_MS1, HIGH);
	pinMode(PIN_MOTOR2_MS1, OUTPUT);
	pinMode(PIN_MOTOR2_MS1, OUTPUT);
	digitalWrite(PIN_MOTOR2_MS1, HIGH);
	digitalWrite(PIN_MOTOR2_MS1, HIGH);
#else
#error Unsupported MICROSTEPPING value
#endif

	pinMode(PIN_MOTOR1_STEP, OUTPUT);
	pinMode(PIN_MOTOR1_DIR, OUTPUT);
	pinMode(PIN_MOTOR2_STEP, OUTPUT);
	pinMode(PIN_MOTOR2_DIR, OUTPUT);
}

// Set speed of Stepper Motor1
// tspeed could be positive or negative (reverse)
void setMotorSpeedM1(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M1 - tspeed) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M1 * 50; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M1 * 25; // 1/8 Microstepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M1 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_M1 = 1;
    digitalWrite(PIN_MOTOR1_DIR, LOW);
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_M1 = -1;
    digitalWrite(PIN_MOTOR1_DIR, HIGH);
  }
  if (timer_period > ZERO_SPEED)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;

  timerAlarmWrite(timer1, timer_period, true);
}

// Set speed of Stepper Motor2
// tspeed could be positive or negative (reverse)
void setMotorSpeedM2(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M2 - tspeed) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M2 * 50; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M2 * 25; // 1/8 Microstepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M2 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_M2 = 1;
    digitalWrite(PIN_MOTOR2_DIR, LOW);
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_M2 = -1;
    digitalWrite(PIN_MOTOR2_DIR, HIGH);
  }
  if (timer_period > ZERO_SPEED)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;

  timerAlarmWrite(timer2, timer_period, true);
}

