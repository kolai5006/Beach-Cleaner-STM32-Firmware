/*
 * drivetrain.c
 *
 *  Two-wheel differential drive on top of motordrivers.c. See drivetrain.h.
 */

#include "drivetrain.h"

static void apply(Drivetrain *dt, int wheel, float pct)
{
    if (pct >  100.0f) pct =  100.0f;
    if (pct < -100.0f) pct = -100.0f;

    float power = (pct < 0.0f) ? -pct : pct;
    uint8_t reverse = (pct < 0.0f) ? 1U : 0U;

    /* fwd_level says which direction bit drives this wheel forwards.
     * Reversing flips it. XOR does both in one step. */
    uint8_t dir = (uint8_t)(dt->fwd_level[wheel] ^ reverse);

    drive_motor(&dt->motor[wheel], power, dir);
}

static HAL_StatusTypeDef setup_wheel(Drivetrain *dt, int wheel,
                                     const WheelConfig *cfg)
{
    dt->fwd_level[wheel]   = cfg->fwd_level ? 1U : 0U;
    dt->has_encoder[wheel] = 0U;

    init_default_motor(&dt->motor[wheel],
                       cfg->pwm_tim, cfg->pwm_channel,
                       cfg->dir_port, cfg->dir_pin);

    /* Stop before anything else can command a speed. */
    drive_motor(&dt->motor[wheel], 0.0f, dt->fwd_level[wheel]);

    if (cfg->enc_tim == NULL) {
        return HAL_OK;
    }

    HAL_StatusTypeDef st = init_encoder(&dt->encoder[wheel],
                                        cfg->enc_tim, cfg->enc_channel);
    if (st == HAL_OK) {
        dt->has_encoder[wheel] = 1U;
    }
    return st;
}

HAL_StatusTypeDef drivetrain_init(Drivetrain *dt,
                                  const WheelConfig *left,
                                  const WheelConfig *right)
{
    if (dt == NULL || left == NULL || right == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef a = setup_wheel(dt, LEFT,  left);
    HAL_StatusTypeDef b = setup_wheel(dt, RIGHT, right);

    return (a == HAL_OK && b == HAL_OK) ? HAL_OK : HAL_ERROR;
}

void drivetrain_set(Drivetrain *dt, float left_pct, float right_pct)
{
    if (dt == NULL) return;
    apply(dt, LEFT,  left_pct);
    apply(dt, RIGHT, right_pct);
}

void drivetrain_stop(Drivetrain *dt)
{
    if (dt == NULL) return;
    apply(dt, LEFT,  0.0f);
    apply(dt, RIGHT, 0.0f);
}

float drivetrain_rpm(const Drivetrain *dt, int wheel)
{
    if (dt == NULL || wheel < 0 || wheel > 1 || !dt->has_encoder[wheel]) {
        return 0.0f;
    }
    return FIT0441_ENCODER_Get_RPM(&dt->encoder[wheel].reading);
}