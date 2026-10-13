/*
 * motordrivers.c
 *
 *  Created on: oct 12, 2025
 *      Author: nicholai
 */
#include "motordrivers.h"

#define ENCODER_HZ 1000000.0f
#define PULSES_PER_REV 270.0f
#define TIMEOUT_STOP 100U
#define MAX_ENCODERS 2U

static Encoder_Driver *encoders[MAX_ENCODERS];

/**
 * Initialize the parameters of a motor struct to default
 * values.
 *
 * @param[in] motor pointer to motor object to initialize.
 * @param[in] fw_t the timer handler for forward driving of the motor.
 * @param[in] fw_channel the timer channel used for forward driving.
 * @param[in] bk_t the timer channel for reverse driving of the motor.
 * @param[in] bk_channel the timer channel used for reverse driving.
 */
void init_default_motor(Motor *RW,
                        TIM_HandleTypeDef *pwm_tim,
                        uint32_t pwm_channel,
                        GPIO_TypeDef *Dir_port,
                        uint16_t Dir_pin)
{
    /*
     * Reaction wheel PWM.
     */
    PWM_Driver RW_pwm = {
            pwm_tim,
            pwm_channel,
            pwm_tim->Init.Period
    };

    /*
     * Fill parameters.
     */
    RW->Motor_t = RW_pwm;
    RW->DirPort = Dir_port;
    RW->DirPin = Dir_pin;
    __HAL_TIM_SET_COMPARE(
        RW_pwm.timer,
        RW_pwm.channel,
        RW_pwm.period + 1U);
    HAL_TIM_PWM_Start(pwm_tim, pwm_channel);

}

void drive_motor(Motor *RW, float power, uint8_t direction)
{
    PWM_Driver RW_pwm = RW->Motor_t;
    uint32_t compare;

    if (power <= 0.0f)
        compare = RW_pwm.period + 1U;
    else if (power >= 100.0f)
        compare = 0U;
    else
        compare = (uint32_t)((RW_pwm.period + 1U) * (1.0f - power * 0.01f));

    HAL_GPIO_WritePin(RW->DirPort, RW->DirPin,
                    direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(
        RW_pwm.timer,
        RW_pwm.channel,
        compare
    );
}

static uint32_t active_channel_flag(uint32_t channel)
{
    switch (channel) {
    case TIM_CHANNEL_1: return HAL_TIM_ACTIVE_CHANNEL_1;
    case TIM_CHANNEL_2: return HAL_TIM_ACTIVE_CHANNEL_2;
    case TIM_CHANNEL_3: return HAL_TIM_ACTIVE_CHANNEL_3;
    case TIM_CHANNEL_4: return HAL_TIM_ACTIVE_CHANNEL_4;
    default:            return 0U;
    }
}

HAL_StatusTypeDef init_encoder(
    Encoder_Driver *encoder,
    TIM_HandleTypeDef *encoder_tim,
    uint32_t encoder_channel)
{
    uint32_t active_channel = active_channel_flag(encoder_channel);
    uint32_t slot = MAX_ENCODERS;

    if (encoder == NULL || encoder_tim == NULL || active_channel == 0U) {
        return HAL_ERROR;
    }

    for (uint32_t i = 0U; i < MAX_ENCODERS; ++i) {
        if (encoders[i] == encoder ||
            (encoders[i] != NULL &&
             encoders[i]->encoder_tim == encoder_tim &&
             encoders[i]->encoder_channel == encoder_channel)) {
            return HAL_ERROR;
        }
        if (encoders[i] == NULL && slot == MAX_ENCODERS) {
            slot = i;
        }
    }
    if (slot == MAX_ENCODERS) {
        return HAL_ERROR;
    }

    encoder->encoder_tim = encoder_tim;
    encoder->encoder_channel = encoder_channel;
    encoder->reading.previousCapture = 0U;
    encoder->reading.havePreviousCapture = 0U;
    encoder->reading.periodTicks = 0U;
    encoder->reading.havePeriod = 0U;
    encoder->reading.lastEdgeMs = 0U;
    encoders[slot] = encoder;

    // Starts the timer's input capture and interrupt.
    HAL_StatusTypeDef status = HAL_TIM_IC_Start_IT(encoder_tim, encoder_channel);
    if (status != HAL_OK) {
        encoders[slot] = NULL;
    }
    return status;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    for (uint32_t i = 0U; i < MAX_ENCODERS; ++i) {
        Encoder_Driver *encoder = encoders[i];
        if (encoder == NULL || encoder->encoder_tim != htim ||
            htim->Channel != active_channel_flag(encoder->encoder_channel)) {
            continue;
        }

        Encoder_Reading *reading = &encoder->reading;
        uint16_t captured = (uint16_t)
            HAL_TIM_ReadCapturedValue(htim, encoder->encoder_channel);

        if (reading->havePreviousCapture) {
            // Modulo subtraction handles one 16-bit timer wrap.
            uint16_t elapsed =
                (uint16_t)(captured - reading->previousCapture);

            if (elapsed != 0U) {
                reading->periodTicks = elapsed;
                reading->havePeriod = 1U;
            }
        }

        reading->previousCapture = captured;
        reading->havePreviousCapture = 1U;
        reading->lastEdgeMs = HAL_GetTick();
        return;
    }
}

float FIT0441_ENCODER_Get_RPM(const Encoder_Reading *reading)
{
    if (reading == NULL) {
        return 0.0f;
    }

    uint16_t period = reading->periodTicks;
    uint32_t last_edge = reading->lastEdgeMs;

    if (!reading->havePeriod ||
        period == 0U ||
        (HAL_GetTick() - last_edge) > TIMEOUT_STOP) {
        return 0.0f;
    }

    return (60.0f * ENCODER_HZ) /
           (PULSES_PER_REV * (float)period);
}

