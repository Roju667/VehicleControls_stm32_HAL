/*
 * parse.c
 *
 *  Created on: 24 cze 2021
 *      Author: ROJEK
 */

#include "main.h"
#include "string.h"
#include "ringbuffer.h"
#include "usart.h"
#include "tim.h"
#include "stdlib.h"
#include "stdio.h"
#include "movement_control.h"
#include "parse.h"

/*
 * @ function parse message and start command procedures
 */
uint8_t Parser_Parse(uint8_t *ParseBuffer, Servo_t *ServoX, Motor_t *MotorY, VehicleLED_t *VehicleLEDs)
{
	uint8_t *ParsePointer;

	ParsePointer = (uint8_t*) (strtok((char*) ParseBuffer, ";"));

	if (strcmp((char*) ParsePointer, "ACK") == 0)
	{
		// light up green led
		VehicleLEDs->ControlWord |= (0x01 << LED_COMM_OFFSET);

		for (uint8_t i = 0; i < 4; i++)
		{

			ParsePointer = (uint8_t*) (strtok(NULL, ";"));

			switch (i)
			{

			case 1: // write servo position
				ServoX->PWMCommandNew = atoi((const char*) ParsePointer);
				break;

			case 2: // write motor speed
				MotorY->PWMCommandNew = atoi((const char*) ParsePointer);
				break;

			case 3: // control LED
				if (strcmp((char*) ParsePointer, "1") == 0)
				{
					// set lights on
					VehicleLEDs->ControlWord |= (0x01 << LED_FRONT1_OFFSET);
					VehicleLEDs->ControlWord |= (0x01 << LED_FRONT2_OFFSET);
				}
				else
				{
					// reset lights
					VehicleLEDs->ControlWord &= ~(0x01 << LED_FRONT1_OFFSET);
					VehicleLEDs->ControlWord &= ~(0x01 << LED_FRONT2_OFFSET);
				}
				break;
			case 4: // control motor

				// check if button motor on is pressed
				// if yes allow movement of servo and motor
				if (strcmp((char*) ParsePointer, "1") == 0)
				{
					ServoX->ContorlOn = 1;
					MotorY->ControlOn = 1;
				}
				else
				{
					ServoX->ContorlOn = 0;
					MotorY->ControlOn = 0;
				}
			}

		}

	}

	return 0;
}
