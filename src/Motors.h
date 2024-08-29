/*
 * Motors.h
 *
 *  Created on: 25.09.2017
 *      Author: anonymous
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include <Arduino.h>

void Motors_setup();
void setMotorSpeedM1(int16_t tspeed);
void setMotorSpeedM2(int16_t tspeed);

#endif /* MOTORS_H_ */
