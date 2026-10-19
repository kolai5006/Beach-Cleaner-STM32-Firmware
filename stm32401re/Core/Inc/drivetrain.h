/*
 * drivetrain.h
 *
 *  Two-wheel differential drive on top of motordrivers.c.
 *
 *  WHY THIS EXISTS
 *    motordrivers.c drives one motor: a power 0..100 and a direction bit.
 *    A differential drive needs a pair, and the two motors are mounted
 *    facing opposite ways, so "forward" is a different direction bit on
 *    each side. Getting that wrong makes the robot spin instead of drive.
 *    This file holds that knowledge in one place so no caller has to.
 *
 *  USAGE
 *      static Drivetrain wheels;
 *
 *      const WheelConfig left_wheel = {
 *          .pwm_tim = &htim3, .pwm_channel = TIM_CHANNEL_1,   // PB4
 *          .dir_port = GPIOB, .dir_pin = GPIO_PIN_10,
 *          .fwd_level = 0,
 *          .enc_tim = &htim4, .enc_channel = TIM_CHANNEL_1,   // PB6
 *      };
 *      const WheelConfig right_wheel = {
 *          .pwm_tim = &htim3, .pwm_channel = TIM_CHANNEL_2,   // PA7
 *          .dir_port = GPIOA, .dir_pin = GPIO_PIN_4,
 *          .fwd_level = 1,            // mirrored motor
 *          .enc_tim = &htim4, .enc_channel = TIM_CHANNEL_2,   // PB7
 *      };
 *
 *      drivetrain_init(&wheels, &left_wheel, &right_wheel);
 *      drivetrain_set(&wheels, 30.0f, 30.0f);   // straight ahead at 30%
 *      drivetrain_stop(&wheels);
 *      float rpm = drivetrain_rpm(&wheels, LEFT);
 *
 *  Speeds are signed percent, -100..100. Negative is reverse. The sign
 *  becomes the direction bit and the magnitude becomes the power, with
 *  fwd_level applied, so the caller only ever thinks in "forward is
 *  positive on both wheels".
 */

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "motordrivers.h"

typedef struct {
    TIM_HandleTypeDef *pwm_tim;
    uint32_t           pwm_channel;
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;
    uint8_t            fwd_level;    /* direction bit meaning forward: 0 or 1 */
    TIM_HandleTypeDef *enc_tim;      /* NULL to skip the encoder */
    uint32_t           enc_channel;
} WheelConfig;

typedef struct {
    Motor          motor[2];         /* 0 = left, 1 = right */
    Encoder_Driver encoder[2];
    uint8_t        fwd_level[2];
    uint8_t        has_encoder[2];
} Drivetrain;

#define LEFT   0
#define RIGHT  1

/* Initialises both motors and (where configured) both encoders, and leaves
 * the drivetrain stopped. Returns HAL_ERROR if an encoder refused to start;
 * the motors are still usable in that case. */
HAL_StatusTypeDef drivetrain_init(Drivetrain *dt,
                                  const WheelConfig *left,
                                  const WheelConfig *right);

/* Signed percent per wheel, -100..100. Values outside are clamped. */
void drivetrain_set(Drivetrain *dt, float left_pct, float right_pct);

/* Both wheels to zero. Call this from your failsafe. */
void drivetrain_stop(Drivetrain *dt);

/* Measured wheel speed, RPM, always positive - the FIT0441 encoder reports
 * speed only, with no direction. */
float drivetrain_rpm(const Drivetrain *dt, int wheel);

#endif /* DRIVETRAIN_H */