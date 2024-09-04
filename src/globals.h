/*
 * globals.h
 *
 *  Created on: 25.09.2017
 *      Author: anonymous
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <Arduino.h>
#include <Preferences.h> // for storing settings in the ESP32 EEPROM
#include "esp32-hal-timer.h"

// -- EEPROM
extern Preferences preferences;

extern long timer_old;

// Angle of the robot (used for stability control)
extern float angle_adjusted;            // +90 degrees leaning forward, -90 degrees leaning backward
extern float angle_adjusted_Old;
extern float angle_adjusted_filtered;

// Default control values from constant definitions
extern float Kp;
extern float Kd;
extern float Kp_thr;
extern float Ki_thr;
extern float Kp_user;
extern float Kd_user;
extern float Kp_thr_user;
extern float Ki_thr_user;
extern float Kp_position;
extern float Kd_position;
extern float PID_errorSum;
extern float PID_errorOld;
extern float PID_errorOld2;
extern float setPointOld;
extern int16_t throttle;
extern float steering;                    // positive == turn left, negative == turn right
extern float max_throttle;
extern float max_steering;
extern float max_target_angle;
extern float control_output;
extern float angle_offset;

extern boolean positionControlMode;
extern uint8_t mode;  // mode = 0 Normal mode, mode = 1 Pro mode (More agressive)

// position control
extern volatile int32_t steps1;
extern volatile int32_t steps2;
extern int32_t target_steps1;
extern int32_t target_steps2;
extern int16_t motor1_control;
extern int16_t motor2_control;

extern int16_t speed_M1, speed_M2;        // Actual speed of motors
extern int8_t  dir_M1, dir_M2;            // Actual direction of steppers motors

#ifdef JJROBOTS_APP
// OSC output variables
extern uint8_t OSCpage;
extern uint8_t OSCnewMessage;
extern float OSCfader[4];
extern float OSCxy1_x;
extern float OSCxy1_y;
extern float OSCxy2_x;
extern float OSCxy2_y;
extern uint8_t OSCpush[4];
extern uint8_t OSCtoggle[4];
extern uint8_t OSCmove_mode;
extern int16_t OSCmove_speed;
extern int16_t OSCmove_steps1;
extern int16_t OSCmove_steps2;
#endif // JJROBOTS_APP

#endif /* GLOBALS_H_ */
