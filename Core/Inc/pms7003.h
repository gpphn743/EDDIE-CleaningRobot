/*
 * pms7003.h
 *
 *  Created on: Dec 19, 2025
 *      Author: ASUS
 */

#ifndef INC_PMS7003_H_
#define INC_PMS7003_H_

#include "stm32f1xx_hal.h"

typedef struct {
    uint16_t pm1_0_std;
    uint16_t pm2_5_std;
    uint16_t pm10_std;
    uint16_t pm1_0_env;
    uint16_t pm2_5_env; // Usually the value you want for air quality
    uint16_t pm10_env;
} PMS_Data_t;

#endif /* INC_PMS7003_H_ */
