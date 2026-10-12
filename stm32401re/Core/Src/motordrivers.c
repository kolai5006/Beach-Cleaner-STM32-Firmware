/*
 * motordrivers.c
 *
 *  Created on: Sep 12, 2025
 *      Author: nicho, adam :p
 */
#include "motordrivers.h"

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

void drive_motor(Motor *RW, float power)
{
    PWM_Driver RW_pwm = RW->Motor_t;
    uint32_t compare;

    if (power <= 0.0f)
        compare = RW_pwm.period + 1U;
    else if (power >= 100.0f)
        compare = 0U;
    else
        compare = (uint32_t)((RW_pwm.period + 1U) * (1.0f - power * 0.01f));

    HAL_GPIO_WritePin(RW->DirPort, RW->DirPin, RESET);
    __HAL_TIM_SET_COMPARE(
        RW_pwm.timer,
        RW_pwm.channel,
        compare
    );
}

