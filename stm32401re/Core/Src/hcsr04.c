/*
 * hcsr04.c
 *
 *  Created on: Oct 13, 2026
 *      Author: nicholai
 */

#include "hcsr04.h"

#define HCSR04_ECHO_TIMEOUT_US 40000U

static uint16_t HCSR04_ElapsedUs(const HCSR04 *sensor, uint16_t start)
{
    return (uint16_t)(__HAL_TIM_GET_COUNTER(sensor->Timer) - start);
}

static uint8_t HCSR04_WaitForEchoLevel(const HCSR04 *sensor,
                                       GPIO_PinState level,
                                       uint16_t timeout_us)
{
    uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(sensor->Timer);

    while (HAL_GPIO_ReadPin(sensor->EchoPort, sensor->EchoPin) != level) {
        if (HCSR04_ElapsedUs(sensor, start) >= timeout_us) {
            return 0U;
        }
    }

    return 1U;
}

HAL_StatusTypeDef HCSR04_Init(HCSR04 *sensor,
                              GPIO_TypeDef *TrigPort, uint16_t TrigPin,
                              GPIO_TypeDef *EchoPort, uint16_t EchoPin,
                              TIM_HandleTypeDef *Timer)
{
    if (sensor == NULL || TrigPort == NULL || EchoPort == NULL ||
        Timer == NULL || Timer->Instance == NULL) {
        return HAL_ERROR;
    }

    sensor->TrigPort = TrigPort;
    sensor->TrigPin = TrigPin;
    sensor->EchoPort = EchoPort;
    sensor->EchoPin = EchoPin;
    sensor->Timer = Timer;

    HAL_GPIO_WritePin(sensor->TrigPort, sensor->TrigPin, GPIO_PIN_RESET);
    return HAL_TIM_Base_Start(sensor->Timer);
}

HAL_StatusTypeDef HCSR04_ReadDistance(HCSR04 *sensor, float *distance_cm)
{
    if (sensor == NULL || distance_cm == NULL || sensor->Timer == NULL) {
        return HAL_ERROR;
    }

    *distance_cm = -1.0f;

    HAL_GPIO_WritePin(sensor->TrigPort, sensor->TrigPin, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(sensor->TrigPort, sensor->TrigPin, GPIO_PIN_SET);

    uint16_t trigger_start = (uint16_t)__HAL_TIM_GET_COUNTER(sensor->Timer);
    while (HCSR04_ElapsedUs(sensor, trigger_start) < 10U) {
    }

    HAL_GPIO_WritePin(sensor->TrigPort, sensor->TrigPin, GPIO_PIN_RESET);

    if (!HCSR04_WaitForEchoLevel(sensor, GPIO_PIN_SET,
                                 HCSR04_ECHO_TIMEOUT_US)) {
        return HAL_TIMEOUT;
    }

    uint16_t echo_start = (uint16_t)__HAL_TIM_GET_COUNTER(sensor->Timer);
    while (HAL_GPIO_ReadPin(sensor->EchoPort, sensor->EchoPin) == GPIO_PIN_SET) {
        if (HCSR04_ElapsedUs(sensor, echo_start) >= HCSR04_ECHO_TIMEOUT_US) {
            return HAL_TIMEOUT;
        }
    }

    uint16_t pulse_width_us = HCSR04_ElapsedUs(sensor, echo_start);
    *distance_cm = ((float)pulse_width_us * 0.0343f) / 2.0f;
    return HAL_OK;
}
