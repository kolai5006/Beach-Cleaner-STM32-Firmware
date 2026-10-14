/*

    author: nicholai

    date: oct 13, 2026s

    description: header file for hcsr04 ultrasonic sensor driver
*/

#pragma once

#include "stm32f4xx_hal.h"

typedef struct {
    GPIO_TypeDef *TrigPort;
    uint16_t TrigPin;
    GPIO_TypeDef *EchoPort;
    uint16_t EchoPin;
    TIM_HandleTypeDef *Timer;
} HCSR04;


HAL_StatusTypeDef HCSR04_Init(HCSR04 *sensor,
                              GPIO_TypeDef *TrigPort, uint16_t TrigPin,
                              GPIO_TypeDef *EchoPort, uint16_t EchoPin,
                              TIM_HandleTypeDef *Timer);
HAL_StatusTypeDef HCSR04_ReadDistance(HCSR04 *sensor, float *distance_cm);
