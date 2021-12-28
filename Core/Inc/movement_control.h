/*
 * movement_control.h
 *
 *  Created on: Dec 21, 2021
 *      Author: pawel
 */
#include "tim.h"

#ifndef INC_MOVEMENTT_CONTROL_H_
#define INC_MOVEMENT_CONTROL_H_

// MOTOR defines
#define	MOTOR_PWM_COUNTERPERIOD 	40000
#define MOTOR_LOW_LIMIT 			0
#define MOTOR_HIGH_LIMIT			MOTOR_PWM_COUNTERPERIOD
#define MOTOR_RESOLUTION_1PERCENT	MOTOR_PWM_COUNTERPERIOD / 100
#define MOTOR_DEADBAND_LOW_LIMIT	18000
#define MOTOR_DEADBAND_HIGH_LIMIT 	22000

// MOTOR GPIOs
#define DIR1_GPIO_PORT				GPIOB
#define DIR2_GPIO_PORT				GPIOB
#define DIR1_PIN					GPIO_PIN_10
#define DIR2_PIN					GPIO_PIN_11

// SERVO defines
#define	SERVO_PWM_COUNTERPERIOD 	40000								// has to be 50 Hz (20ms) - (0,5ms = 0 degree) - (2,5ms = 180 degree)
#define SERVO_LOW_LIMIT_DEGREES		60									// lowest position for servo
#define SERVO_HIGH_LIMIT_DEGREES 	120									// highest position for servo
#define SERVO_0_DEGREES_PWM			SERVO_PWM_COUNTERPERIOD / 40		// starting value is 2,5% of 50Hz PWM signal
#define SERVO_RESOLUTION_1DEGREE	SERVO_PWM_COUNTERPERIOD / 1800		// 1 degree is 22 adc points
#define SERVO_LOW_LIMIT 			((SERVO_0_DEGREES_PWM) + (SERVO_LOW_LIMIT_DEGREES * (SERVO_PWM_COUNTERPERIOD / 1800)))		// calculate it to PWM values
#define SERVO_HIGH_LIMIT			((SERVO_0_DEGREES_PWM) + (SERVO_HIGH_LIMIT_DEGREES * (SERVO_PWM_COUNTERPERIOD / 1800)))		// calculate it to PWM values

// MOTOR driver commands
#define LN298N_MOTOR_OFF()			HAL_GPIO_WritePin(DIR1_GPIO_PORT, DIR1_PIN, GPIO_PIN_RESET);HAL_GPIO_WritePin(DIR2_GPIO_PORT, DIR2_PIN, GPIO_PIN_RESET)
#define LN298N_MOTOR_FORWARD()		HAL_GPIO_WritePin(DIR1_GPIO_PORT, DIR1_PIN, GPIO_PIN_SET);HAL_GPIO_WritePin(DIR2_GPIO_PORT, DIR2_PIN, GPIO_PIN_RESET)
#define LN298N_MOTOR_BACKWARD()		HAL_GPIO_WritePin(DIR1_GPIO_PORT, DIR1_PIN, GPIO_PIN_RESET);HAL_GPIO_WritePin(DIR2_GPIO_PORT, DIR2_PIN, GPIO_PIN_SET)

// Motor struct
typedef struct Motor
{
	uint16_t PWMCommandNew; 		// new value recieved from joystick cmd (0-40000)

	uint16_t PWMCommandLast;		// value from last joystick cmd (0 - 40000)

	uint16_t MotorSpeed;			// value that is set on PWM (0 - 40000)

	TIM_HandleTypeDef* PWMTimer;	// timer for PWM

	uint8_t PWMTimerChannel;		// timer channel for PWM

}Motor_t;

// Servo struct
typedef struct Servo
{
	uint16_t PWMCommandNew; 		// new value recieved from joystick cmd (0-40000)

	uint16_t PWMCommandLast;		// value from last joystick cmd (0 - 40000)

	TIM_HandleTypeDef* PWMTimer;	// timer for PWM

	uint8_t PWMTimerChannel;		// timer channel for PWM

}Servo_t;

// functions
void Motor_Control (Motor_t *Motor);
void Motor_Init (Motor_t* Motor, TIM_HandleTypeDef* PWMTimer, uint8_t PWMTimerChannel);
void Servo_Control (Servo_t *Servo);
void Servo_Init (Servo_t *Servo, TIM_HandleTypeDef* PWMTimer, uint8_t PWMTimerChannel);

#endif /* INC_MOVEMENT_CONTROL_H_ */
