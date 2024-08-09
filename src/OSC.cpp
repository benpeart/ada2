/*
 * OSC.cpp
 *
 *  Created on: 02.10.2017
 *      Author: anonymous
 */

#include "OSC.h"
#include <WiFiUdp.h>
#include "debug.h"
#include "globals.h"
#include "defines.h"
#include "Motors.h"

// for DEBUG uncomment this lines...
//#define OSCDEBUG 0

#define MAX_BUFFER 32
unsigned char UDPBuffer[MAX_BUFFER]; // input message buffer

// OSC message internal variables
unsigned char OSCtouchMessage;
WiFiUDP udp;
IPAddress telemetryServer(192, 168, 4, 2);

// ------- OSC functions -----------------------------------------

// Aux functions
float OSC_extractParamFloat(uint8_t pos) {
	union {
		unsigned char Buff[4];
		float d;
	} u;

	u.Buff[0] = UDPBuffer[pos + 3];
	u.Buff[1] = UDPBuffer[pos + 2];
	u.Buff[2] = UDPBuffer[pos + 1];
	u.Buff[3] = UDPBuffer[pos];

	return (u.d);
}

int16_t OSC_extractParamInt(uint8_t pos) {
	union {
		unsigned char Buff[2];
		int16_t d;
	} u;

	u.Buff[1] = UDPBuffer[pos + 1];
	u.Buff[0] = UDPBuffer[pos];

	return (u.d);
}

void OSC_init() {
	udp.begin(2222);

	OSCfader[0] = 0.5;
	OSCfader[1] = 0.5;
	OSCfader[2] = 0.5;
	OSCfader[3] = 0.5;
}

void OSC_MsgSend(char *c, unsigned char msgSize, float p) {
	uint8_t i;
	union {
		unsigned char Buff[4];
		float d;
	} u;

	// We copy the param in the last 4 bytes
	u.d = p;
	c[msgSize - 4] = u.Buff[3];
	c[msgSize - 3] = u.Buff[2];
	c[msgSize - 2] = u.Buff[1];
	c[msgSize - 1] = u.Buff[0];

	OSC_MsgSend(c, msgSize);
}

void OSC_MsgSend(char *c, unsigned char msgSize) {
	udp.beginPacket(telemetryServer, 2223);
	udp.write((uint8_t*) c, msgSize);
	udp.endPacket();
}

void OSC_MsgRead() {
	int packetSize = udp.parsePacket();

	if (packetSize <= MAX_BUFFER && packetSize > 0) {
		if (udp.read(UDPBuffer, MAX_BUFFER)) {
#ifdef OSCDEBUG
			Serial.println(UDPBuffer);
#endif
			// We look for an OSC message start like /x/
			if ((UDPBuffer[0] == '/') && (UDPBuffer[2] == '/')
					&& ((UDPBuffer[1] == '1') || (UDPBuffer[1] == '2'))) {
				OSCnewMessage = 1;
				OSCpage = UDPBuffer[1] - '0';  // Convert page to int
				OSCtouchMessage = 0;

				float value = OSC_extractParamFloat(16);

				if ((UDPBuffer[5] == 'd') && (UDPBuffer[6] == 'e')
						&& (UDPBuffer[7] == 'r')) {
					// Fader    /1/fader1 ,f  xxxx

					OSCfader[UDPBuffer[8] - '0' - 1] = value;
					return;
				} else if ((UDPBuffer[4] == 'u') && (UDPBuffer[5] == 's')
						&& (UDPBuffer[6] == 'h')) {
					// Push message

					if (value == 0)
						OSCpush[UDPBuffer[7] - '0' - 1] = 0;
					else
						OSCpush[UDPBuffer[7] - '0' - 1] = 1;

					return;
				} else if ((UDPBuffer[6] == 'g') && (UDPBuffer[7] == 'l')
						&& (UDPBuffer[8] == 'e')) {
					// Toggle message
					if (value == 0)
						OSCtoggle[UDPBuffer[9] - '0' - 1] = 0;
					else
						OSCtoggle[UDPBuffer[9] - '0' - 1] = 1;
					return;
				}
			}
		}
	}
}

void OSC_MsgProcess()
{
	if (OSCpage == 1)
	{
		if (modifing_control_parameters) // We came from the settings screen
		{
			DB_PRINTLN("OSC modifing_control_parameters");
			OSCfader[0] = 0.5; // default neutral values
			OSCfader[1] = 0.5;
			OSCtoggle[0] = 0; // Normal mode
			mode = 0;
			modifing_control_parameters = false;
		}

		if (OSCmove_mode)
		{
#ifdef DEBUG_OSC
			DB_PRINTLN("OSCmove_mode positionControlMode = true");
			DB_PRINT("OSCmove_speed:");
			DB_PRINT(OSCmove_speed);
			DB_PRINT(",");
			DB_PRINT("OSCmove_steps1:");
			DB_PRINT(OSCmove_steps1);
			DB_PRINT(",");
			DB_PRINT("OSCmove_steps2:");
			DB_PRINTLN(OSCmove_steps2);
#endif
			positionControlMode = true;
			OSCmove_mode = false;
			target_steps1 = steps1 + OSCmove_steps1;
			target_steps2 = steps2 + OSCmove_steps2;
		}
		else
		{
			positionControlMode = false;
			throttle = (OSCfader[0] - 0.5) * max_throttle;
			// We add some exponential on steering to smooth the center band
			steering = OSCfader[1] - 0.5;
			if (steering > 0)
				steering = (steering * steering + 0.5 * steering) * max_steering;
			else
				steering = (-steering * steering + 0.5 * steering) * max_steering;
#ifdef DEBUG_OSC
			DB_PRINT(", throttle_input:");
			DB_PRINT(OSCfader[0]);
			DB_PRINT(", OSC throttle:");
			DB_PRINT(throttle);
			DB_PRINT(", steering_input:");
			DB_PRINT(OSCfader[1]);
			DB_PRINT(", OSC steering:");
			DB_PRINT(steering);
#endif
		}

		if ((mode == 0) && (OSCtoggle[0]))
		{
			// Change to PRO mode
			DB_PRINTLN("OSC Change to PRO mode");
			max_throttle = MAX_THROTTLE_PRO;
			max_steering = MAX_STEERING_PRO;
			max_target_angle = MAX_TARGET_ANGLE_PRO;
			mode = 1;
		}
		if ((mode == 1) && (OSCtoggle[0] == 0))
		{
			// Change to NORMAL mode
			DB_PRINTLN("OSC Change to NORMAL mode");
			max_throttle = MAX_THROTTLE;
			max_steering = MAX_STEERING;
			max_target_angle = MAX_TARGET_ANGLE;
			mode = 0;
		}
	}
	else if (OSCpage == 2)
	{ // OSC page 2
		if (!modifing_control_parameters)
		{
			for (uint8_t i = 0; i < 4; i++)
				OSCfader[i] = 0.5;
			OSCtoggle[0] = 0;

			modifing_control_parameters = true;
			char buf[] = "$P2";
			OSC_MsgSend(buf, 4);
		}
		// User could adjust KP, KD, KP_THROTTLE and KI_THROTTLE (fadder1,2,3,4)
		// Now we need to adjust all the parameters all the times because we dont know what parameter has been moved
		Kp_user = KP * 2 * OSCfader[0];
		Kd_user = KD * 2 * OSCfader[1];
		Kp_thr_user = KP_THROTTLE * 2 * OSCfader[2];
		Ki_thr_user = KI_THROTTLE * 2 * OSCfader[3];
		// Send a special telemetry message with the new parameters
		char auxS[50];
		sprintf(auxS, "$tP,%d,%d,%d,%d", int(Kp_user * 1000),
				int(Kd_user * 1000), int(Kp_thr_user * 1000),
				int(Ki_thr_user * 1000));
		OSC_MsgSend(auxS, 50);

#ifdef DEBUG_OSC
		DB_PRINTLN("OSC modifing control parameters");
		DB_PRINT("Kp_user:");
		DB_PRINT(Kp_user);
		DB_PRINT(",");
		DB_PRINT("Kd_user:");
		DB_PRINT(Kd_user);
		DB_PRINT(",");
		DB_PRINT("Kp_thr_user:");
		DB_PRINT(Kp_thr_user);
		DB_PRINT(",");
		DB_PRINT("Ki_thr_user:");
		DB_PRINTLN(Ki_thr_user);
#endif

		// Calibration mode??
		if (OSCpush[2] == 1)
		{
			DB_PRINT("OSC Calibration MODE ");
			angle_offset = angle_adjusted_filtered;
			DB_PRINTLN(angle_offset);
		}

		// Kill robot => Sleep
		while (OSCtoggle[0] == 1)
		{
			DB_PRINTLN("OSC Sleep");
			// Reset external parameters
			PID_errorSum = 0;
			timer_old = millis();
			setMotorSpeedM1(0);
			setMotorSpeedM2(0);
			digitalWrite(PIN_ENABLE_MOTORS, HIGH); // Disable motors
			OSC_MsgRead();
		}
	}
}
