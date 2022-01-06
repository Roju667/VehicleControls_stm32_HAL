/*
 * movement_control.c
 *
 *  Created on: Dec 21, 2021
 *      Author: pawel
 */

#include "movement_control.h"
#include "stdlib.h"
#include "gpio.h"

/*
 * Motor controls function :
 * - check if value is not in a deadband
 * - check if value changed by 1%
 * - check if movement if forward/backward
 *
 * @param[*Motor] - handler of Motor struct
 * @return - void
 */
void Motor_Control(Motor_t *Motor)
{

	if (Motor->ControlOn == 1)
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
		else if (abs(
				Motor->PWMCommandNew
						- Motor->PWMCommandLast) > MOTOR_RESOLUTION_1PERCENT)
		{
			// deadbands used to avoid soft movements and noise
			if (Motor->PWMCommandNew < MOTOR_DEADBAND_LOW_LIMIT)
			{
				// write gpio to LN298 for backward movement
				LN298N_MOTOR_BACKWARD();
				// even if joystick is moved backward, PWM value has to be more than high deadband
				// beacuse direction is set by GPIO
				// 40000 - newcommand (max speed = 40000 - 0)
				Motor->MotorSpeed = (MOTOR_PWM_COUNTERPERIOD
						- Motor->PWMCommandNew);
			}
			// forward movement
			else
			{
				// write gpio for forward movement
				LN298N_MOTOR_FORWARD();
				// value from joystick is equal to speed value for forward movement
				Motor->MotorSpeed = Motor->PWMCommandNew;
			}
			__HAL_TIM_SET_COMPARE(Motor->PWMTimer, Motor->PWMTimerChannel,
					Motor->MotorSpeed);
		}
	}
	// if button is not pressed turn off engine and return
	else
	{
		__HAL_TIM_SET_COMPARE(Motor->PWMTimer, Motor->PWMTimerChannel, 0);
		return;
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
void Motor_Init(Motor_t *Motor, TIM_HandleTypeDef *PWMTimer,
		uint8_t PWMTimerChannel)
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
void Servo_Control(Servo_t *Servo)
{
	// check if button is clicked
	if (Servo->ContorlOn)
	{
		// move only when value changes by 1 degree
		if (abs(
				Servo->PWMCommandNew
						- Servo->PWMCommandLast) > SERVO_RESOLUTION_1DEGREE)
		{
			// write new value to PWM
			__HAL_TIM_SET_COMPARE(Servo->PWMTimer, Servo->PWMTimerChannel,
					Servo->PWMCommandNew);
		}
	}
	else
	{
		// if button is not pressed return
		return;
	}

	// save value for next cycle to compare
	Servo->PWMCommandLast = Servo->PWMCommandNew;
}

/*
 * Assign timer to Servo structure
 *
 * @param[*Servo] - handler of Servo struct
 * @param[*PWMTimer] - handler to HAL timer structure
 * @param[PWMTimerChannel] - channel definition : TIM_CHANNEL_1
 * @return - void
 */
void Servo_Init(Servo_t *Servo, TIM_HandleTypeDef *PWMTimer,
		uint8_t PWMTimerChannel)
{
	// assign parameters to strcut
	Servo->PWMTimer = PWMTimer;
	Servo->PWMTimerChannel = PWMTimerChannel;

	// start PWM singal
	HAL_TIM_PWM_Start(Servo->PWMTimer, Servo->PWMTimerChannel);
}

/*
 * Turn on and off LED by checking contorl word
 *
 * @param[*VehicleLED] - handler of LED struct
 * @return - void
 */
void LED_Control(VehicleLED_t *VehicleLED)
{
	// led COMM
	if (VehicleLED->ControlWord & (0x01 << LED_COMM_OFFSET))
	{
		HAL_GPIO_WritePin(LED_COMM_GPIO_PORT, LED_COMM_PIN, GPIO_PIN_SET);

	}
	else
	{
		HAL_GPIO_WritePin(LED_COMM_GPIO_PORT, LED_COMM_PIN, GPIO_PIN_RESET);
	}

	// led FRONT1
	if (VehicleLED->ControlWord & (0x01 << LED_FRONT1_OFFSET))
	{
		HAL_GPIO_WritePin(LED_FRONT1_GPIO_PORT, LED_FRONT1_PIN, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_FRONT1_GPIO_PORT, LED_FRONT1_PIN, GPIO_PIN_RESET);
	}

	// led FRONT2
	if (VehicleLED->ControlWord & (0x01 << LED_FRONT2_OFFSET))
	{
		HAL_GPIO_WritePin(LED_FRONT2_GPIO_PORT, LED_FRONT2_PIN, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_FRONT2_GPIO_PORT, LED_FRONT2_PIN, GPIO_PIN_RESET);
	}
}

void Vehicle_Shutdown(VehicleLED_t *VehicleLED, Servo_t *Servo, Motor_t *Motor)
{

		// turn off LEDs
		HAL_GPIO_WritePin(LED_COMM_GPIO_PORT, LED_COMM_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_FRONT1_GPIO_PORT, LED_FRONT1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_FRONT2_GPIO_PORT, LED_FRONT2_PIN, GPIO_PIN_RESET);

		//set servo to middle position
		__HAL_TIM_SET_COMPARE(Servo->PWMTimer, Servo->PWMTimerChannel,
				(SERVO_LOW_LIMIT + SERVO_HIGH_LIMIT)/2);

		//turn off motor
		__HAL_TIM_SET_COMPARE(Motor->PWMTimer, Motor->PWMTimerChannel, 0);

}
