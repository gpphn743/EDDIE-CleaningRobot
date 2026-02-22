/*
 * wheel_control.h
 *
 *  Created on: Jan 28, 2026
 *      Author: ASUS
 */

// GPIO Configurations
//ENA - TIM1 - CHANNEL 1 (PA8)
//ENB - TIM1 - CHANNEL 4 (PA11)
//IN1, IN2, IN3, IN4 - PA0, PA1, PC14, PC15

#ifndef INC_L298N_H_
#define INC_L298N_H_

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim1;

 uint32_t tim_arr(void);
 void MotorA_SetSpeed(uint8_t duty);
 void MotorA_SetDir(uint8_t reverse);
 void MotorA_Brake(void);
 void MotorA_Coast(void);
 void MotorB_SetSpeed(uint8_t duty);
 void MotorB_SetDir(uint8_t reverse);
 void MotorB_Brake(void);
 void MotorB_Coast(void);
  void GoForward(uint8_t duty);
  void StopCoast(void);
  void TurnLeft(uint8_t duty);
  void TurnRight(uint8_t duty);
  void GoBack(uint8_t duty);

#endif /* INC_L298N_H_ */
