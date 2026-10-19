/*
 * testing.h
 *
 *  Created on: Oct 17, 2026
 *      Author: nicholai
 *
 *  Bench tests for each peripheral, used to keep main.c clean.
 *  Not used in the final product.
 *
 *  HOW TO USE (in main.c):
 *    1. Call the test's *_Init() once in USER CODE BEGIN 2.
 *    2. Call the test's *_Run() inside the while(1) loop (USER CODE BEGIN 3).
 *    3. Open Live Expressions in the debugger and watch the variables
 *       listed for that test (or a serial monitor at 115200 8N1).
 *
 *  Tests can be combined, e.g. Test_Motor + Test_Ultrasonic, as long as
 *  their pins don't overlap (see the pin table in testing.c).
 */

#pragma once

#include "stm32f4xx_hal.h"
#include "bno055.h"
#include "mcp9808.h"

/* ---- Live-watch variables (add these to Live Expressions) ---- */
extern volatile float test_motor_rpm;               // Test_Motor
extern volatile float test_ultrasonic_distance_cm;  // Test_Ultrasonic
extern volatile HAL_StatusTypeDef test_ultrasonic_status;
extern MCP_9808 test_temp_sensor;                   // Test_TempSensor
extern volatile uint8_t test_temp_status;
extern BNO055_Axis_Vec_t test_gyro_data;            // Test_IMU
extern volatile BNO055_ERROR test_imu_status;
extern volatile uint32_t test_esp32_good;           // Test_ESP32
extern volatile uint32_t test_esp32_bad_crc;
extern volatile uint32_t test_esp32_lost;
extern volatile int8_t test_esp32_motor_speed;      // Test_ESP32_Motor
extern volatile uint8_t test_esp32_failsafe;

/* ---- Motor + encoder (FIT0441) ---- */
void Test_Motor_Init(void);
void Test_Motor_Run(void);

/* ---- HC-SR04 ultrasonic distance ---- */
void Test_Ultrasonic_Init(void);
void Test_Ultrasonic_Run(void);

/* ---- MCP9808 temperature ---- */
void Test_TempSensor_Init(void);
void Test_TempSensor_Run(void);

/* ---- BNO055 IMU gyro ---- */
void Test_IMU_Init(void);
void Test_IMU_Run(void);

/* ---- UART echo over ST-LINK virtual COM port ---- */
void Test_UartEcho_Init(void);
void Test_UartEcho_Run(void);

/* ---- On-board LED blink (LD2) ---- */
void Test_LED_Init(void);
void Test_LED_Run(void);

/* ---- ESP32 to STM32 Communication ---- */
void Test_ESP32_Init(void);
void Test_ESP32_Run(void);

/* ---- ESP32 -> STM32 motor speed (typed in ESP32 serial monitor) ---- */
void Test_ESP32_Motor_Init(void);
void Test_ESP32_Motor_Run(void);