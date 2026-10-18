/*
 * motordrivers.h
 *
 *  Created on: oct 12, 2025
 *      Author: nicholai
 */

/**
 * Motor driving abstractions.
 */

#pragma once

#include "stm32f4xx_hal.h"

/**
 * Represents a timer PWM driver.
 */
typedef struct
{
    /**
     * ST TIM handler.
     */
    TIM_HandleTypeDef *timer;

    /**
     * The timer channel to output on.
     */
    uint32_t channel;

    /**
     * The timer period.
     */
    uint32_t period;
} PWM_Driver;

/**
 * Represents a motor.
 */
typedef struct
{
    /**
     * The timer PWM for Motor.
     */
    PWM_Driver Motor_t;

    /**
     * the Direction Port and pin GPIO port
     */
    GPIO_TypeDef *DirPort;


    uint16_t DirPin;
} Motor;

typedef struct {
    uint16_t previousCapture;
    uint8_t havePreviousCapture;

    volatile uint16_t periodTicks;
    volatile uint8_t havePeriod;
    volatile uint32_t lastEdgeMs;  // Must be 32-bit for HAL_GetTick()
} Encoder_Reading;

typedef struct {
    TIM_HandleTypeDef *encoder_tim;
    uint32_t encoder_channel;
    Encoder_Reading reading;
} Encoder_Driver;


//rw - reactionwheel
void init_default_motor(Motor *RW,
                        TIM_HandleTypeDef *pwm_tim,
                        uint32_t pwm_channel,
                        GPIO_TypeDef *Dir_port,
                        uint16_t Dir_pin);

/**
 * Drive the PWM of the motor.
 *
 * @param[in] motor a pointer to the motor object to drive.
 * @param[in] power motor power from 0 (off) to 100 (full power).
 *
 */
void drive_motor(Motor *RW, float power, uint8_t direction);

HAL_StatusTypeDef init_encoder(
    Encoder_Driver *encoder,
    TIM_HandleTypeDef *encoder_tim,
    uint32_t encoder_channel);

float FIT0441_ENCODER_Get_RPM(const Encoder_Reading *reading);
