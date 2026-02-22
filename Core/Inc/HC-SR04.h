/*
 * HC-SR04.h
 *
 *  Created on: Jan 28, 2026
 *      Author: ASUS
 */

#ifndef INC_HC_SR04_H_
#define INC_HC_SR04_H_

#include "stm32f1xx_hal.h"

#define TRIG1_GPIO_Port		GPIOB
#define TRIG1_Pin			GPIO_PIN_13
#define TRIG2_GPIO_Port		GPIOB
#define TRIG2_Pin			GPIO_PIN_14
#define TRIG3_GPIO_Port		GPIOB
#define TRIG3_Pin			GPIO_PIN_15

extern TIM_HandleTypeDef htim4;

void HCSR04_Trigger(uint8_t channel);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
float HCSR04_Read(uint8_t channel);

#endif /* INC_HC_SR04_H_ */
