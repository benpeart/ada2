/*
 * defines.h
 *
 *  Created on: 25.09.2017
 *      Author: anonymous
 */

#ifndef DEFINES_H_
#define DEFINES_H_

// turn on/off various features
// #define OUTPUT_DT
// #define OLD_MPU6050
// #define SERVO
// #define JJROBOTS_APP
#define WEBUI
#define ELEGANTOTA

// debugging support
// #define DEBUG_IMU
// #define DEBUG_ESTIMATED_SPEED
// #define DEBUG_SPEED
// #define DEBUG_OSC
// #define DEBUG_XBOX_CONTROLLER
// #define XBOX_SERIAL_PLOTTER

#ifdef SERVO
#define PIN_SERVO 23
#endif // SERVO

#define MPU_INTERRUPT 23

#define PIN_BATTERY_VOLTAGE 36 // ADC pin connected to voltage divider

#define PIN_MOTORS_ENABLE 19 // enable

#define PIN_MOTOR1_MS1 18  // left address/subdivision Settings
#define PIN_MOTOR1_MS2 05  // left address/subdivision Settings
#define PIN_MOTOR1_STEP 33 // step pulse input
#define PIN_MOTOR1_DIR 32  // direction input

#define PIN_MOTOR2_MS1 04  // right address/subdivision Settings
#define PIN_MOTOR2_MS2 27  // right address/subdivision Settings
#define PIN_MOTOR2_STEP 26 // step pulse input
#define PIN_MOTOR2_DIR 25  // direction input

#define TELEMETRY "192.168.4.1" // Default telemetry server (first client) port 2223

// NORMAL MODE PARAMETERS (MAXIMUN SETTINGS)
#define MAX_THROTTLE 550
#define MAX_STEERING 140
#define STEERING_DEADZONE_RADIUS (MAX_STEERING / 40) // deadzone radius
#define MAX_TARGET_ANGLE 14

// PRO MODE = MORE AGGRESSIVE (MAXIMUN SETTINGS)
#define MAX_THROTTLE_PRO 780    // Max recommended value: 860
#define MAX_STEERING_PRO 260    // Max recommended value: 280
#define MAX_TARGET_ANGLE_PRO 26 // Max recommended value: 32

// Default control terms for Ada
#define KP 0.32 // 0.32
#define KD 0.08 // 0.050
#define KP_THROTTLE 0.080
#define KI_THROTTLE 0.1
#define KP_POSITION 0.06
#define KD_POSITION 0.45
// #define KI_POSITION 0.02

// Control gains for raiseup (the raiseup movement requiere special control parameters)
#define KP_RAISEUP 0.1
#define KD_RAISEUP 0.16
#define KP_THROTTLE_RAISEUP 0 // No speed control on raiseup
#define KI_THROTTLE_RAISEUP 0.0

#define MAX_CONTROL_OUTPUT 1000
#define ITERM_MAX_ERROR 30 // Iterm windup constants for PI control
#define ITERM_MAX 10000

#define ANGLE_OFFSET 0.0 // Offset angle for balance (to compensate robot own weight distribution)

// Servo definitions
#ifdef SERVO
#define SERVO_AUX_NEUTRO 4444 // Servo neutral position
#define SERVO_MIN_PULSEWIDTH SERVO_AUX_NEUTRO + 3000
#define SERVO_MAX_PULSEWIDTH SERVO_AUX_NEUTRO - 3000

#define SERVO2_NEUTRO 4444
#define SERVO2_RANGE 8400
#endif // SERVO

// Telemetry
#define TELEMETRY_BATTERY 0
#define TELEMETRY_ANGLE 1
// #define TELEMETRY_DEBUG 1  // Dont use TELEMETRY_ANGLE and TELEMETRY_DEBUG at the same time!

#define ZERO_SPEED 0xffffff
#define MAX_ACCEL 20 // Maximun motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)

#define MICROSTEPPING 16 // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)

// now controlled in platform.ini based on build type (debug vs release)
// #define DEBUG 1   // 0 = No debug info (default) DEBUG 1 for console output

// AUX definitions
#define CLR(x, y) (x &= (~(1 << y)))
#define SET(x, y) (x |= (1 << y))
#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#endif /* DEFINES_H_ */
