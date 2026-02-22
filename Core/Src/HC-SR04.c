/*
 * HC-SR04.c
 *
 *  Created on: Jan 28, 2026
 *      Author: ASUS
 */

#include "HC-SR04.h"

volatile uint32_t ic_val1;
volatile uint32_t ic_val2;
volatile uint8_t  is_first_captured;
volatile uint8_t  done_flag;
volatile uint32_t diff;
volatile float    distance_cm;

void HCSR04_Trigger(uint8_t channel)
{
  // channel: 1 => TRIG1, 2 => TRIG2, 3 => TRIG3
  if (channel < 1 || channel > 3) return;

  GPIO_TypeDef* port = NULL;
  uint16_t pin = 0;

  if (channel == 1) { port = TRIG1_GPIO_Port; pin = TRIG1_Pin; }
  else if (channel == 2) { port = TRIG2_GPIO_Port; pin = TRIG2_Pin; }
  else { port = TRIG3_GPIO_Port; pin = TRIG3_Pin; }

  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  // ~10us pulse (ước lượng simple). Nếu có DWT_Delay_us(10) thì dùng nó.
  for (volatile int i = 0; i < 120; i++) __NOP();
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM4) return;

  uint8_t idx = 0;
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) idx = 0;
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) idx = 1;
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) idx = 2;
  else return;

  uint32_t cap;

  if (idx == 0) cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
  else if (idx == 1) cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
  else cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

  if (is_first_captured == 0) {
    ic_val1 = cap;
    is_first_captured = 1;
    // set polarity to falling for this channel
    if (idx == 0) __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    else if (idx == 1) __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
    else __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
  } else {
    ic_val2 = cap;
    // compute diff (handle overflow)
    if (ic_val2 > ic_val1) diff = ic_val2 - ic_val1;
    else diff = (htim4.Init.Period - ic_val1) + ic_val2 + 1;
    distance_cm = (float)diff * 0.0343f / 2.0f;
    is_first_captured = 0;
    // restore polarity to rising and disable interrupt for this channel
    if (idx == 0) {
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
      __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
    } else if (idx == 1) {
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
      __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);
    } else {
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
      __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC3);
    }
  }
}


float HCSR04_Read(uint8_t channel)
{
  if (channel < 1 || channel > 3) return -1.0f;

  // reset trạng thái trước đo
  is_first_captured = 0;
  ic_val1 = 0;
  ic_val2 = 0;
  diff = 0;
  distance_cm = 0.0f;

  // set capture polarity RISING & enable IT cho kênh
  if (channel == 1) {
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
  } else if (channel == 2) {
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC2);
  } else {
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC3);
  }

  // Trigger sau khi đã sẵn sàng capture
  HCSR04_Trigger(channel);

  // chờ callback cập nhật distance_cm, timeout ~100ms
  uint32_t start = HAL_GetTick();
  while (distance_cm == 0.0f) {
    if (HAL_GetTick() - start > 100) {
      // timeout: disable IT cho kênh tương ứng
      if (channel == 1)      __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
      else if (channel == 2) __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC2);
      else                   __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC3);
      return -1.0f;
    }
  }

  // callback của bạn đã tự disable IT (theo code hiện tại)
  return distance_cm;
}


