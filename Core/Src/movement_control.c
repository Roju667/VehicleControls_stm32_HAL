/*
 * movement_control.c
 *
 *  Created on: Dec 21, 2021
 *      Author: pawel
 */


#include "movement_control.h"
#include "stdlib.h"


/*
 * Motor controls function :
 * - check if value is not in a deadband
 * - check if value changed by 1%
 * - check if movement if forward/backward
 *
 * @param[*Motor] - handler of Motor struct
 * @return - void
 */
void Motor_Control (Motor_t *Motor)
{

	// check if value is not within deadband limits
	if (Motor->PWMCommandNew > MOTOR_DEADBAND_LOW_LIMIT
			&& Motor->PWMCommandNew < MOTOR_DEADBAND_HIGH_LIMIT)
	{
		// if value is in deadband set PWM to 0%
		__HAL_TIM_SET_COMPARE(Motor->PWMTimer, Motor->PWMTimerChannel, 0);
		// turn GPIOs to Motor off
		LN298N_MOTOR_OFF();
	}
	//check if adc value changed by 1%
	else if (abs(Motor->PWMCommandNew - Motor->PWMCommandLast) > MOTOR_RESOLUTION_1PERCENT)
	{
			// deadbands use to avoid soft movements and noise
			if (Motor->PWMCommandNew < MOTOR_DEADBAND_LOW_LIMIT)
			{
				// write gpio to LN298 for backward movement
				LN298N_MOTOR_BACKWARD();
				// even if joystick is moved backward, PWM value has to be more than high deadband
				// beacuse direction is set by GPIO
				Motor->MotorSpeed = Motor->PWMCommandNew + MOTOR_DEADBAND_LOW_LIMIT;
			}
			else
			{
				// write gpio for forward movement
				LN298N_MOTOR_FORWARD();
				// value from joystick is equal to speed value for forward movement
				Motor->MotorSpeed = Motor->PWMCommandNew;
			}

			__HAL_TIM_SET_COMPARE(Motor->PWMTimer, Motor->PWMTimerChannel, Motor->MotorSpeed);
	}

	// save value for next cycle to compare
	Motor->PWMCommandLast = Motor->PWMCommandNew;
}

/*
 * Assign timer to Motor structure
 *
 * @param[*Motor] - handler of Motor struct
 * @param[*PWMTimer] - handler to HAL timer structure
 * @param[PWMTimerChannel] - channel definition : TIM_CHANNEL_1
 * @return - void
 */
void Motor_Init (Motor_t* Motor, TIM_HandleTypeDef* PWMTimer, uint8_t PWMTimerChannel)
{
	// assign parameters to strcut
	Motor->PWMTimer = PWMTimer;
	Motor->PWMTimerChannel = PWMTimerChannel;

	// start PWM singal
	HAL_TIM_PWM_Start(Motor->PWMTimer, Motor->PWMTimerChannel);
}


/*
 *	Servo controls function :
 * - check if value differs by 1% from last cycle
 *
 * @param[*Servo] - handler of Motor struct
 * @return - void
 */
void Servo_Control (Servo_t *Servo)
{
	// move only when value changes by 1 degree
	if (abs(Servo->PWMCommandNew - Servo->PWMCommandLast) > SERVO_RESOLUTION_1DEGREE)
	{
		// write new value to PWM
		__HAL_TIM_SET_COMPARE(Servo->PWMTimer, Servo->PWMTimerChannel, Servo->PWMCommandNew);
	}

	// save value for next cycle to compare
	Servo->PWMCommandLast = Servo->PWMCommandNew;
}

/*
 * Assign timer to Motor structure
 *
 * @param[*Servo] - handler of Servo struct
 * @param[*PWMTimer] - handler to HAL timer structure
 * @param[PWMTimerChannel] - channel definition : TIM_CHANNEL_1
 * @return - void
 */
void Servo_Init (Servo_t *Servo, TIM_HandleTypeDef* PWMTimer, uint8_t PWMTimerChannel)
{
	// assign parameters to strcut
	Servo->PWMTimer = PWMTimer;
	Servo->PWMTimerChannel = PWMTimerChannel;

	// start PWM singal
	HAL_TIM_PWM_Start(Servo->PWMTimer, Servo->PWMTimerChannel);
}
