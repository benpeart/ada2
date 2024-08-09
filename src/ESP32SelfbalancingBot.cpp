// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS CONTROLLED WITH YOUR SMARTPHONE
// JJROBOTS BROBOT KIT: (Arduino Leonardo + BROBOT ELECTRONIC BRAIN SHIELD + STEPPER MOTOR drivers)
// This code is prepared for the ESP32
// Author: JJROBOTS.COM
// Date: 02/09/2014
// Updated: 25/06/2017
// Version: 2.82
// License: GPL v2
// Compiled and tested with Arduino 1.6.8. This new version of code does not need external libraries (only Arduino standard libraries)
// Project URL: http://jjrobots.com/b-robot-evo-2-much-more-than-a-self-balancing-robot (Features,documentation,build instructions,how it works, SHOP,...)
// New updates:
//   - New default parameters specially tuned for BROBOT EVO 2 version (More agile, more stable...)
//   - New Move mode with position control (for externally programming the robot with a Blockly or pyhton programming interfaces)
//   - New telemtry packets send to TELEMETRY IP for monitoring Battery, Angle, ... (old battery packets for touch osc not working now)
//   - Default telemetry server is 192.168.4.2 (first client connected to the robot)
//  Get the free android app (jjrobots) from google play. For IOS users you need to use TouchOSC App + special template (info on jjrobots page)
//  Thanks to our users on the forum for the new ideas. Specially sasa999, KomX, ...

// The board needs at least 10-15 seconds with no motion (robot steady) at beginning to give good values... Robot move slightly when it´s ready!
// MPU6050 IMU connected via I2C bus. Angle estimation using complementary filter (fusion between gyro and accel)
// Angle calculations and control part is running at 100Hz

// The robot is OFF when the angle is high (robot is horizontal). When you start raising the robot it
// automatically switch ON and start a RAISE UP procedure.
// You could RAISE UP the robot also with the robot arm servo (Servo button on the interface)
// To switch OFF the robot you could manually put the robot down on the floor (horizontal)

// We use a standard PID controllers (Proportional, Integral derivative controller) for robot stability
// More info on the project page: How it works page at jjrobots.com
// We have a PI controller for speed control and a PD controller for stability (robot angle)
// The output of the control (motors speed) is integrated so it´s really an acceleration not an speed.

// We control the robot from a WIFI module using OSC standard UDP messages
// You need an OSC app to control de robot (Custom free JJRobots APP for android, and TouchOSC APP for IOS)
// Join the module Wifi Access Point (by default: JJROBOTS_XX) with your Smartphone/Tablet...
//   Wifi password: 87654321
// For TouchOSC users (IOS): Install the BROBOT layout into the OSC app (Touch OSC) and start play! (read the project page)
// OSC controls:
//    fader1: Throttle (0.0-1.0) OSC message: /1/fader1
//    fader2: Steering (0.0-1.0) OSC message: /1/fader2
//    push1: Move servo arm (and robot raiseup) OSC message /1/push1
//    if you enable the touchMessage on TouchOSC options, controls return to center automatically when you lift your fingers
//    PRO mode (PRO button). On PRO mode steering and throttle are more aggressive
//    PAGE2: PID adjustements [optional][dont touch if you dont know what you are doing...;-) ]

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include "MPU6050.h"
#include <stdio.h>
#include "esp_types.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "defines.h"
#include "globals.h"
#include "Motors.h"
#include "Control.h"
#include <WiFiUdp.h>
#include "OSC.h"
#include "esp32-hal-ledc.h"
#include "debug.h"
#include "xbox.h"

void initTimers();

void initWifiAP()
{
	DB_PRINTLN("Setting up WiFi AP...");
	if (WiFi.softAP("bbot", "12345678"))
	{
		DB_PRINTLN("Wifi AP set up successfully");
	}
	WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
}

void initMPU6050()
{
	MPU6050_setup();
	delay(500);
	MPU6050_calibrate();
}

void setup()
{
	pinMode(PIN_ENABLE_MOTORS, OUTPUT);
	digitalWrite(PIN_ENABLE_MOTORS, HIGH);

	pinMode(PIN_MOTOR1_DIR, OUTPUT);
	pinMode(PIN_MOTOR1_STEP, OUTPUT);
	pinMode(PIN_MOTOR2_DIR, OUTPUT);
	pinMode(PIN_MOTOR2_STEP, OUTPUT);

	pinMode(PIN_SERVO, OUTPUT);

	ledcSetup(6, 50, 16);		 // channel 6, 50 Hz, 16-bit width
	ledcAttachPin(PIN_SERVO, 6); // GPIO 22 assigned to channel 1
	delay(50);
	ledcWrite(6, SERVO_AUX_NEUTRO);

	Serial.begin(115200);

	Wire.begin();

	initWifiAP();

	initMPU6050();
	initTimers();

	Xbox_setup();
	OSC_init();

	// move the motors back and forth to indicate life
	digitalWrite(PIN_ENABLE_MOTORS, LOW);
	for (uint8_t k = 0; k < 5; k++)
	{
		setMotorSpeedM1(5);
		setMotorSpeedM2(5);
		ledcWrite(6, SERVO_AUX_NEUTRO + 250);
		delay(200);
		setMotorSpeedM1(-5);
		setMotorSpeedM2(-5);
		ledcWrite(6, SERVO_AUX_NEUTRO - 250);
		delay(200);
	}
	ledcWrite(6, SERVO_AUX_NEUTRO);

	digitalWrite(PIN_ENABLE_MOTORS, HIGH);
}

void loop()
{
	OSC_MsgRead();
	if (OSCnewMessage)
	{
		OSCnewMessage = 0;
		OSC_MsgProcess();
	}

	Xbox_loop();

	timer_value = micros();

	if (MPU6050_newData())
	{
		MPU6050_read_3axis();

		loop_counter++;
		slow_loop_counter++;
		dt = (timer_value - timer_old) * 0.000001; // dt in seconds
		timer_old = timer_value;

		angle_adjusted_Old = angle_adjusted;
		// Get new orientation angle from IMU (MPU6050)
		float MPU_sensor_angle = MPU6050_getAngle(dt);
		angle_adjusted = MPU_sensor_angle + angle_offset;
		if ((MPU_sensor_angle > -15) && (MPU_sensor_angle < 15))
			angle_adjusted_filtered = angle_adjusted_filtered * 0.99 + MPU_sensor_angle * 0.01;

#ifdef DEBUG_IMU
		DB_PRINT(", MPU_sensor_angle:");
		DB_PRINT(MPU_sensor_angle);

		DB_PRINT(", angle_offset:");
		DB_PRINT(angle_offset);

		DB_PRINT(", angle_adjusted:");
		DB_PRINT(angle_adjusted);

		DB_PRINT(", angle_adjusted_filtered:");
		DB_PRINT(angle_adjusted_filtered);
#endif // DEBUG_IMU
	   // We calculate the estimated robot speed:
	   // Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
		actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward

		int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0; // 25 is an empirical extracted factor to adjust for real units
		int16_t estimated_speed = -actual_robot_speed + angular_velocity;
		estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float)estimated_speed * 0.1; // low pass filter on estimated speed

#ifdef DEBUG_ESTIMATED_SPEED
		DB_PRINT(", speed_M1:");
		DB_PRINT(speed_M1);
		DB_PRINT(", speed_M2:");
		DB_PRINT(speed_M2);
		DB_PRINT(", actual_robot_speed:");
		DB_PRINT(actual_robot_speed);
		DB_PRINT(", angular_velocity:");
		DB_PRINT(angular_velocity);
		DB_PRINT(", estimated_speed_filtered:");
		DB_PRINT(estimated_speed_filtered);
#endif // DEBUG_ESTIMATED_SPEED

		if (positionControlMode)
		{
			// POSITION CONTROL. INPUT: Target steps for each motor. Output: motors speed
			motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
			motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

			// Convert from motor position control to throttle / steering commands
			throttle = (motor1_control + motor2_control) / 2;
			throttle = constrain(throttle, -190, 190);
			steering = motor2_control - motor1_control;
			steering = constrain(steering, -50, 50);
		}

		// ROBOT SPEED CONTROL: This is a PI controller.
		//    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
		target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
		target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output

#ifdef DEBUG_SPEED
		DB_PRINT(", angle_adjusted:");
		DB_PRINT(angle_adjusted);
		DB_PRINT(", estimated_speed_filtered:");
		DB_PRINT(estimated_speed_filtered);
		DB_PRINT(", target_angle:");
		DB_PRINT(target_angle);
#endif // DEBUG_SPEED

		// Stability control (100Hz loop): This is a PD controller.
		//    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
		//    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
		control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
		control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

		// The steering part from the user is injected directly to the output
		motor1 = control_output - steering;
		motor2 = control_output + steering;

		// Limit max speed (control output)
		motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
		motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

		int angle_ready;
		if (OSCpush[0]) // If we press the SERVO button we start to move
			angle_ready = 82;
		else
			angle_ready = 74;												   // Default angle
		if ((angle_adjusted < angle_ready) && (angle_adjusted > -angle_ready)) // Is robot ready (upright?)
		{
			// NORMAL MODE
			digitalWrite(PIN_ENABLE_MOTORS, LOW); // Motors enable
			// NOW we send the commands to the motors
			setMotorSpeedM1(motor1);
			setMotorSpeedM2(motor2);
		}
		else // Robot not ready (flat), angle > angle_ready => ROBOT OFF
		{
			digitalWrite(PIN_ENABLE_MOTORS, HIGH); // Disable motors
			setMotorSpeedM1(0);
			setMotorSpeedM2(0);
			PID_errorSum = 0; // Reset PID I term
			Kp = KP_RAISEUP;  // CONTROL GAINS FOR RAISE UP
			Kd = KD_RAISEUP;
			Kp_thr = KP_THROTTLE_RAISEUP;
			Ki_thr = KI_THROTTLE_RAISEUP;
			// RESET steps
			steps1 = 0;
			steps2 = 0;
			positionControlMode = false;
			OSCmove_mode = false;
			throttle = 0;
			steering = 0;
		}
		// Push1 Move servo arm
		if (OSCpush[0]) // Move arm
		{
			if (angle_adjusted > -40)
				ledcWrite(6, SERVO_MIN_PULSEWIDTH);
			else
				ledcWrite(6, SERVO_MAX_PULSEWIDTH);
		}
		else
			ledcWrite(6, SERVO_AUX_NEUTRO);

		// Servo2
		// ledcWrite(6, SERVO2_NEUTRO + (OSCfader[2] - 0.5) * SERVO2_RANGE);

		// Normal condition?
		if ((angle_adjusted < 56) && (angle_adjusted > -56))
		{
			Kp = Kp_user; // Default user control gains
			Kd = Kd_user;
			Kp_thr = Kp_thr_user;
			Ki_thr = Ki_thr_user;
		}
		else // We are in the raise up procedure => we use special control parameters
		{
			Kp = KP_RAISEUP; // CONTROL GAINS FOR RAISE UP
			Kd = KD_RAISEUP;
			Kp_thr = KP_THROTTLE_RAISEUP;
			Ki_thr = KI_THROTTLE_RAISEUP;
		}
#ifdef DEBUG_IMU		
		DB_PRINTLN("");
#endif // DEBUG_IMU		
	} // End of new IMU data

	// Medium loop 7.5Hz
	if (loop_counter >= 15)
	{
		loop_counter = 0;
		// Telemetry here?
#if TELEMETRY_ANGLE == 1
		char auxS[25];
		int ang_out = constrain(int(angle_adjusted * 10), -900, 900);
		sprintf(auxS, "$tA,%+04d", ang_out);
		OSC_MsgSend(auxS, 25);
#endif
#if TELEMETRY_DEBUG == 1
		char auxS[50];
		sprintf(auxS, "$tD,%d,%d,%ld", int(angle_adjusted * 10), int(estimated_speed_filtered), steps1);
		OSC_MsgSend(auxS, 50);
#endif
	} // End of medium loop
	else if (slow_loop_counter >= 100) // 1Hz
	{
		slow_loop_counter = 0;
		// Read  status
#if TELEMETRY_BATTERY == 1
		BatteryValue = (BatteryValue + BROBOT_readBattery(false)) / 2;
		sendBattery_counter++;
		if (sendBattery_counter >= 3)
		{ // Every 3 seconds we send a message
			sendBattery_counter = 0;
			DB_PRINT("BatteryValue:");
			DB_PRINTLN(BatteryValue);
			char auxS[25];
			sprintf(auxS, "$tB,%04d", BatteryValue);
			OSC_MsgSend(auxS, 25);
		}
#endif
	} // End of slow loop
}
