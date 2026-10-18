/*
 * testing.c
 *
 *  Created on: Oct 17, 2026
 *      Author: nicholai
 *
 *  Bench tests for each peripheral, used to declutter and organize main.c.
 *  Not used in the final product. See testing.h for how to use.
 *
 *  PIN TABLE (Nucleo-F401RE)
 *  ---------------------------------------------------------------
 *  Test              Signal          Pin    Peripheral
 *  ---------------------------------------------------------------
 *  Test_Motor        PWM             PA0    TIM2_CH1 (~20 kHz)
 *                    Direction       PB10   GPIO output
 *                    Encoder (FG)    PB6    TIM4_CH1 input capture
 *  Test_Ultrasonic   Trigger         PB5    GPIO output
 *                    Echo            PA8    TIM1 (1 us tick)
 *  Test_TempSensor   SCL / SDA       PB8 / PB9   I2C1 (addr 0x18)
 *  Test_IMU          SCL / SDA       PB8 / PB9   I2C1 (shared bus)
 *  Test_UartEcho     TX / RX         PA2 / PA3   USART2 -> ST-LINK USB
 *  Test_LED          Toggle          PA5    GPIO output
 *  ---------------------------------------------------------------
 *  Spare, configured but unused: PB4 (TIM3_CH1 capture, 2nd encoder),
 *  PA4 / PA5 (GPIO out), PA1 (GPIO in).
 */

#include "testing.h"

#include <stdio.h>
#include "main.h"           // Error_Handler()
#include "motordrivers.h"
#include "hcsr04.h"

/* Peripheral handles created by CubeMX in main.c */
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;

/* Live-watch variables (declared in testing.h) */
volatile float test_motor_rpm = 0.0f;
volatile float test_ultrasonic_distance_cm = -1.0f;
volatile HAL_StatusTypeDef test_ultrasonic_status = HAL_ERROR;
MCP_9808 test_temp_sensor;
volatile uint8_t test_temp_status = 0xFF;
BNO055_Axis_Vec_t test_gyro_data;
volatile BNO055_ERROR test_imu_status = BNO055_NOT_DETECTED;


/* =====================================================================
 * MOTOR + ENCODER TEST (FIT0441 brushless motor)
 *
 *  Pins:  PA0  -> motor PWM   (TIM2_CH1)
 *         PB10 -> motor direction
 *         PB6  <- encoder FG  (TIM4_CH1 input capture)
 *
 *  Spins the motor at TEST_MOTOR_POWER % in TEST_MOTOR_DIRECTION and
 *  reads speed from the encoder.
 *  Watch: test_motor_rpm
 * ===================================================================== */
#define TEST_MOTOR_POWER      50.0f  // 0 (off) to 100 (full)
#define TEST_MOTOR_DIRECTION  0U     // 0 or 1

static Motor test_motor;
static Encoder_Driver test_encoder;

void Test_Motor_Init(void)
{
    init_default_motor(&test_motor, &htim2, TIM_CHANNEL_1, GPIOB, GPIO_PIN_10);
    init_encoder(&test_encoder, &htim4, TIM_CHANNEL_1);
}

void Test_Motor_Run(void)
{
    drive_motor(&test_motor, TEST_MOTOR_POWER, TEST_MOTOR_DIRECTION);
    test_motor_rpm = FIT0441_ENCODER_Get_RPM(&test_encoder.reading);
}


/* =====================================================================
 * HC-SR04 ULTRASONIC TEST
 *
 *  Pins:  PB5 -> TRIG
 *         PA8 <- ECHO  (through a voltage divider, 5 V -> 3.3 V)
 *
 *  Takes one reading every 60 ms (sensor needs >= 60 ms between pings).
 *  Watch: test_ultrasonic_distance_cm (-1 = no echo),
 *         test_ultrasonic_status (HAL_OK / HAL_TIMEOUT)
 * ===================================================================== */
static HCSR04 test_ultrasonic;

void Test_Ultrasonic_Init(void)
{
    if (HCSR04_Init(&test_ultrasonic, GPIOB, GPIO_PIN_5,
                    GPIOA, GPIO_PIN_8, &htim1) != HAL_OK) {
        Error_Handler();
    }
}

void Test_Ultrasonic_Run(void)
{
    float distance_cm = -1.0f;
    test_ultrasonic_status = HCSR04_ReadDistance(&test_ultrasonic, &distance_cm);
    test_ultrasonic_distance_cm = distance_cm;
    HAL_Delay(60U);
}


/* =====================================================================
 * MCP9808 TEMPERATURE TEST
 *
 *  Pins:  PB8 -> SCL, PB9 <-> SDA  (I2C1, 100 kHz, address 0x18)
 *
 *  Reads ambient temperature every 500 ms.
 *  Watch: test_temp_sensor.temperatureC / .temperatureF,
 *         test_temp_status (0 = OK, 1 = I2C read failed)
 * ===================================================================== */
void Test_TempSensor_Init(void)
{
    uint8_t status = MCP_9808_Init(&hi2c1);

    if (status != 0) {
        printf("[Warning] MCP9808 not detected (status=%d)\r\n", status);
    } else {
        printf("[OK] MCP9808 initialized\r\n");
    }
}

void Test_TempSensor_Run(void)
{
    test_temp_status = MCP_9808_ReadTemp(&test_temp_sensor);
    HAL_Delay(500U);
}


/* =====================================================================
 * BNO055 IMU TEST
 *
 *  Pins:  PB8 -> SCL, PB9 <-> SDA  (I2C1, shared with MCP9808)
 *
 *  Puts the IMU in IMU (accel + gyro fusion) mode and reads the gyro.
 *  Init takes ~1.5 s because of required BNO055 reset delays.
 *  Watch: test_gyro_data.x / .y / .z, test_imu_status (0 = OK)
 * ===================================================================== */
void Test_IMU_Init(void)
{
    BNO055_I2C_Mount(&hi2c1);
    test_imu_status = BNO055_Init();

    if (test_imu_status != BNO055_SUCCESS) {
        printf("[Warning] BNO055 not detected (status=%d)\r\n", test_imu_status);
        return;
    }

    BNO055_Set_OP_Mode(IMU);
    printf("[OK] BNO055 initialized\r\n");
}

void Test_IMU_Run(void)
{
    test_imu_status = BNO055_Get_Gyro(&test_gyro_data);
    HAL_Delay(10U);
}


/* =====================================================================
 * UART ECHO TEST
 *
 *  Pins:  PA2 -> TX, PA3 <- RX  (USART2, wired to the ST-LINK USB
 *                                virtual COM port; no extra wiring)
 *
 *  Open a serial monitor at 115200 8N1. Typed characters are echoed
 *  back; pressing Enter prints the whole line.
 * ===================================================================== */
static uint8_t rx_byte = 0;
static char rx_buffer[64];
static uint8_t rx_index = 0;

void Test_UartEcho_Init(void)
{
    printf("\r\n========================================\r\n");
    printf("   STM32F401RE UART Echo Monitor Online!\r\n");
    printf("   Baud Rate: 115200 8N1\r\n");
    printf("========================================\r\n");
    printf("Ready! Type characters and press Enter:\r\n> ");
}

void Test_UartEcho_Run(void)
{
    // Non-blocking: wait at most 10 ms for one byte
    if (HAL_UART_Receive(&huart2, &rx_byte, 1, 10) != HAL_OK) {
        return;
    }

    // Echo so you can see what you type
    HAL_UART_Transmit(&huart2, &rx_byte, 1, 10);

    if (rx_byte == '\r' || rx_byte == '\n') {
        if (rx_index > 0) {
            rx_buffer[rx_index] = '\0';
            printf("\r\n[STM32 Received]: \"%s\"\r\n> ", rx_buffer);
            rx_index = 0;
        }
    } else if (rx_index < sizeof(rx_buffer) - 1) {
        rx_buffer[rx_index++] = (char)rx_byte;
    }
}


/* =====================================================================
 * LED BLINK TEST
 *
 *  Pin:  PA5 -> LED (LD2, on-board)
 *
 *  The LED will blink at a regular interval.
 * ===================================================================== */
void Test_LED_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void Test_LED_Run(void)
{
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(500);
}