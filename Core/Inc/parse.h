/*
 * parse.h
 *
 *  Created on: 24 cze 2021
 *      Author: ROJEK
 */

#ifndef INC_PARSE_H_
#define INC_PARSE_H_

uint8_t Parser_Parse(uint8_t *ParseBuffer, Servo_t *ServoX, Motor_t *MotorY,
		VehicleLED_t *VehicleLEDs);

#endif /* INC_PARSE_H_ */
