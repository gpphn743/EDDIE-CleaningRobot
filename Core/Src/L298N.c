/*
 * wheel_control.c
 *
 *  Created on: Jan 28, 2026
 *      Author: ASUS
 */

#include "L298N.h"

 inline uint32_t tim_arr(void) {
  return __HAL_TIM_GET_AUTORELOAD(&htim1);   // expect 3599
}

/* ---- Motor A ---- */
 void MotorA_SetSpeed(uint8_t duty)
{
  if (duty > 100) duty = 100;
  uint32_t arr = tim_arr();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (arr + 1) * duty / 100);
}

 void MotorA_SetDir(uint8_t reverse)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, reverse ? GPIO_PIN_SET 	 : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, reverse ? GPIO_PIN_RESET  : GPIO_PIN_SET);
}

 void MotorA_Brake(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
}

 void MotorA_Coast(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
}

/* ---- Motor B ---- */
 void MotorB_SetSpeed(uint8_t duty)
{
  if (duty > 100) duty = 100;
  uint32_t arr = tim_arr();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (arr + 1) * duty / 100);
}

 void MotorB_SetDir(uint8_t reverse)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, reverse ? GPIO_PIN_SET   : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, reverse ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

 void MotorB_Brake(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
}

 void MotorB_Coast(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
}

 inline void GoForward(uint8_t duty){
  MotorA_SetDir(1); MotorB_SetDir(1);
  MotorA_SetSpeed(duty); MotorB_SetSpeed(duty);
}

/* Dừng trôi cả hai bánh */
 inline void StopCoast(void){
  MotorA_SetSpeed(0); MotorB_SetSpeed(0);
  MotorA_Coast();     MotorB_Coast();
}

/* Rẽ trái kiểu swing: dừng bánh trái (A), chạy bánh phải (B) tiến */
 inline void TurnLeft(uint8_t duty)
{
  MotorA_SetSpeed(0);
  MotorA_Brake();      // hãm A cho đứng hẳn
  MotorB_SetDir(1);    // B tiến
  MotorB_SetSpeed(duty);
}

/* Rẽ phải kiểu swing: dừng bánh phải (B), chạy bánh trái (A) tiến */
 inline void TurnRight(uint8_t duty)
{
  MotorB_SetSpeed(0);
  MotorB_Brake();      // hãm B cho đứng hẳn
  MotorA_SetDir(1);    // A tiến
  MotorA_SetSpeed(duty);
}

/* Lùi: cả 2 bánh đảo chiều và chạy với duty */
 inline void GoBack(uint8_t duty)
{
	MotorA_SetSpeed(0);
	MotorB_SetSpeed(0);
  MotorA_Coast();
	MotorB_Coast();
  MotorA_SetDir(0);
  MotorB_SetDir(0);
  MotorA_SetSpeed(duty);
  MotorB_SetSpeed(duty);
}
