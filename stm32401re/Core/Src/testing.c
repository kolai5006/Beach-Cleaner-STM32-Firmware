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
 *  Test_ESP32        TX / RX         PA9 / PA10  USART1 -> ESP32 (D8 / D2)
 *  Test_ESP32_Motor  ESP32 + motor   Test_ESP32 pins + Test_Motor pins
 *  ---------------------------------------------------------------
 *  Spare, configured but unused: PB4 (TIM3_CH1 capture, 2nd encoder),
 *  PA4 / PA5 (GPIO out), PA1 (GPIO in).
 */

#include "testing.h"

#include <stdio.h>
#include "main.h"           // Error_Handler()
#include "motordrivers.h"
#include "hcsr04.h"
#include "esp32comm.h"
#include "drivetrain.h"

/* Peripheral handles created by CubeMX in main.c */
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
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
    init_default_motor(&test_motor, &htim3, TIM_CHANNEL_1, GPIOB, GPIO_PIN_10);
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

/* Below this line are tests, that implement multiple functionalities/sensors */

/* =====================================================================
 * ESP32 -> STM32 COMMUNICATION TEST
 *
 *  Wiring (both boards are 3.3 V logic):
 *      ESP32 GPIO25 (TX)  ->  PA10 (USART1_RX, Arduino D2)
 *      ESP32 GPIO26 (RX)  <-  PA9  (USART1_TX, Arduino D8)
 *      ESP32 GND          --  STM32 GND   (must share ground)
 *
 *  USART1 (115200 8N1, interrupt on) talks to the ESP32.
 *  USART2 stays on the ST-LINK USB, so printf shows up on the PC.
 *
 *  Packet format + receive logic live in esp32comm.c (see esp32comm.h).
 *  ESP32 sketch: uart_commtest/uartcomm/uartcomm.ino
 *
 *  The ESP32 sends PING packets (AA 55 01 01 <seq> <crc>) at 20 Hz.
 *  Every good ping is echoed back, so the ESP32's "echoed back" count
 *  should read 120 bytes/s (6 bytes x 20 packets).
 *
 *  Open a serial monitor on the ST-LINK COM port at 115200. Each packet
 *  prints one line, e.g.
 *      [ESP32] seq= 42  crc OK  | good=43 bad_crc=0 lost=0 junk=0
 *  Watch: test_esp32_good, test_esp32_bad_crc, test_esp32_lost
 * ===================================================================== */
extern UART_HandleTypeDef huart1;

volatile uint32_t test_esp32_good = 0;
volatile uint32_t test_esp32_bad_crc = 0;
volatile uint32_t test_esp32_lost = 0;

void Test_ESP32_Init(void)
{
    printf("\r\n========================================\r\n");
    printf("   ESP32 -> STM32 UART ping test (USART1)\r\n");
    printf("   ESP32 TX(25) -> PA10, ESP32 RX(26) <- PA9\r\n");
    printf("========================================\r\n");
    printf("Waiting for packets...\r\n");

    if (ESP32_Comm_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

void Test_ESP32_Run(void)
{
    static uint8_t last_seq = 0;
    static uint8_t have_last_seq = 0;
    ESP32_Packet pkt;

    while (ESP32_Comm_GetPacket(&pkt)) {
        if (pkt.type != ESP32_TYPE_PING || pkt.len != 1U) {
            continue;  // not a ping, ignore
        }

        uint8_t seq = pkt.payload[0];

        if (pkt.crc_ok) {
            // Count skipped sequence numbers (uint8_t math handles 255 -> 0)
            if (have_last_seq) {
                test_esp32_lost += (uint8_t)(seq - last_seq - 1U);
            }
            last_seq = seq;
            have_last_seq = 1;

            ESP32_Comm_SendPacket(ESP32_TYPE_PING, &seq, 1);  // echo back
        }

        const ESP32_Stats *stats = ESP32_Comm_GetStats();
        test_esp32_good = stats->good;
        test_esp32_bad_crc = stats->bad_crc;

        printf("[ESP32] seq=%3u  crc %s | good=%lu bad_crc=%lu lost=%lu junk=%lu\r\n",
               seq, pkt.crc_ok ? "OK " : "BAD",
               stats->good, stats->bad_crc, test_esp32_lost, stats->junk);
    }
}


/* =====================================================================
 * ESP32 -> STM32 MOTOR SPEED TEST
 *
 *  Type a speed into the ESP32's serial monitor and the motor follows it.
 *  ESP32 sketch: uart_commtest/motorcmd/motorcmd.ino
 *
 *  Pins:  same ESP32 wiring as the ping test (USART1, PA9 / PA10)
 *         PA0  -> motor PWM   (TIM2_CH1)
 *         PB10 -> motor direction
 *         PB6  <- encoder FG  (TIM4_CH1)
 *
 *  The ESP32 resends the last speed as a MOTOR packet 10 times a second:
 *      AA 55 02 01 <speed> <crc>     speed: int8, -100..100
 *  Positive = direction 0, negative = direction 1, 0 = stop.
 *
 *  FAILSAFE: if no good MOTOR packet arrives for ESP32_MOTOR_TIMEOUT_MS,
 *  the motor stops until packets come back. Test it by typing "x" in the
 *  ESP32 monitor (stops sending) or unplugging the ESP32 TX wire.
 *
 *  Serial monitor (ST-LINK, 115200) prints when the speed changes, when
 *  the failsafe trips, and a status line once a second.
 *  Watch: test_esp32_motor_speed, test_motor_rpm, test_esp32_failsafe
 *
 *  Don't run this together with Test_Motor (they share the motor).
 * ===================================================================== */
#define ESP32_MOTOR_TIMEOUT_MS  500U
#define ESP32_MOTOR_STATUS_MS   1000U

volatile int8_t test_esp32_motor_speed = 0;
volatile uint8_t test_esp32_failsafe = 1;  // 1 = stopped, no fresh commands

void Test_ESP32_Motor_Init(void)
{
    init_default_motor(&test_motor, &htim3, TIM_CHANNEL_1, GPIOB, GPIO_PIN_10);
    init_encoder(&test_encoder, &htim4, TIM_CHANNEL_1);
    drive_motor(&test_motor, 0.0f, 0U);

    printf("\r\n========================================\r\n");
    printf("   ESP32 -> STM32 motor speed test\r\n");
    printf("   Type -100..100 in the ESP32 serial monitor\r\n");
    printf("   Failsafe: motor stops after %u ms of silence\r\n",
           (unsigned)ESP32_MOTOR_TIMEOUT_MS);
    printf("========================================\r\n");
    printf("Waiting for motor commands...\r\n");

    if (ESP32_Comm_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

void Test_ESP32_Motor_Run(void)
{
    static uint32_t last_cmd_ms = 0;
    static uint32_t last_status_ms = 0;
    ESP32_Packet pkt;

    // 1. Take in every packet that arrived and keep the newest speed
    while (ESP32_Comm_GetPacket(&pkt)) {
        if (!pkt.crc_ok || pkt.type != ESP32_TYPE_MOTOR || pkt.len != 1U) {
            continue;
        }

        int8_t speed = (int8_t)pkt.payload[0];
        if (speed > 100) {
            speed = 100;
        } else if (speed < -100) {
            speed = -100;
        }

        if (test_esp32_failsafe) {
            printf("[MOTOR] commands received, failsafe cleared\r\n");
            test_esp32_failsafe = 0;
        }
        if (speed != test_esp32_motor_speed) {
            printf("[MOTOR] new speed %d%%\r\n", speed);
        }

        test_esp32_motor_speed = speed;
        last_cmd_ms = HAL_GetTick();
    }

    // 2. Failsafe: stop if the commands went quiet
    if (!test_esp32_failsafe && (HAL_GetTick() - last_cmd_ms) > ESP32_MOTOR_TIMEOUT_MS) {
        test_esp32_failsafe = 1;
        printf("[FAILSAFE] no command for %u ms, motor stopped\r\n",
               (unsigned)ESP32_MOTOR_TIMEOUT_MS);
    }

    // 3. Act on the stored speed
    int8_t speed = test_esp32_failsafe ? 0 : test_esp32_motor_speed;
    float power = (speed < 0) ? (float)(-speed) : (float)speed;
    drive_motor(&test_motor, power, (speed < 0) ? 1U : 0U);
    test_motor_rpm = FIT0441_ENCODER_Get_RPM(&test_encoder.reading);

    // 4. Status line once a second
    if ((HAL_GetTick() - last_status_ms) >= ESP32_MOTOR_STATUS_MS) {
        last_status_ms = HAL_GetTick();
        const ESP32_Stats *stats = ESP32_Comm_GetStats();
        // nano printf has no %f, so print rpm as a whole number
        printf("[STATUS] speed=%4d%%  rpm=%4d  %s | good=%lu bad_crc=%lu junk=%lu\r\n",
               speed, (int)test_motor_rpm,
               test_esp32_failsafe ? "FAILSAFE" : "running ",
               stats->good, stats->bad_crc, stats->junk);
    }
}

/* =====================================================================
 * DRIVETRAIN TEST (two motors, differential drive)
 *
 *  Pins (confirmed against stm32f4xx_hal_msp.c):
 *
 *         LEFT  wheel          RIGHT wheel
 *         PB4  PWM  TIM3_CH1   PA7  PWM  TIM3_CH2
 *         PB10 direction       PA4  direction
 *         PB6  enc   TIM4_CH1  PB7  enc   TIM4_CH2
 *
 *  Both PWMs share TIM3, so 50% means the same duty on each wheel.
 *  Both encoders share TIM4 at prescaler 83 (1 us tick), which is what
 *  ENCODER_HZ in motordrivers.c assumes.
 *
 *  Note the PWM is ACTIVE LOW - drive_motor() writes compare = period+1
 *  for 0% (pin high, motor stopped) and 0 for 100% (pin low). That is
 *  correct for the FIT0441; do not "fix" it.
 *
 *  RUN THESE IN ORDER. The first one tells you how to configure the
 *  second, and running the second without it can send the robot
 *  backwards across the room.
 *
 *  1. Test_Drivetrain_Wheels_Run()
 *       Drives ONE wheel at a time, forwards, and prints which.
 *       Put the robot ON BLOCKS so the wheels spin free.
 *       Watch each wheel. If a wheel turns the WRONG way, change
 *       that side's .fwd_level below from 0 to 1 (or 1 to 0),
 *       rebuild, and run again until both turn forwards.
 *
 *  2. Test_Drivetrain_Run()
 *       Cycles through every movement: forward, back, spin left,
 *       spin right, arc left, arc right, with a stop between each.
 *       Keep it on blocks the first time, then try the floor.
 *
 *  Serial monitor on the ST-LINK port, 115200. Each step prints when
 *  it starts, plus wheel RPM once a second.
 *  Watch: test_left_rpm, test_right_rpm, test_move
 * ===================================================================== */

extern TIM_HandleTypeDef htim3;   /* PWM, both wheels  */
extern TIM_HandleTypeDef htim4;   /* encoders, both wheels */

#define DRIVE_SPEED      30.0f   /* % for straight moves  */
#define TURN_SPEED       30.0f   /* % for spins           */
#define ARC_SPEED  15.0f   /* % inner wheel on arcs */

static Drivetrain wheels;

volatile float   test_left_rpm  = 0.0f;
volatile float   test_right_rpm = 0.0f;
volatile uint8_t test_move      = 0;

/* Set .fwd_level from what you see in Test_Drivetrain_Wheels_Run.
 * The two motors are mirrored, so these are usually NOT the same. */
static const WheelConfig left_wheel = {
    .pwm_tim     = &htim3,
    .pwm_channel = TIM_CHANNEL_1,
    .dir_port    = GPIOB,
    .dir_pin     = GPIO_PIN_10,
    .fwd_level   = 0U,
    .enc_tim     = &htim4,
    .enc_channel = TIM_CHANNEL_1,
};

static const WheelConfig right_wheel = {
    .pwm_tim     = &htim3,
    .pwm_channel = TIM_CHANNEL_2,
    .dir_port    = GPIOA,
    .dir_pin     = GPIO_PIN_4,
    .fwd_level   = 1U,
    .enc_tim     = &htim4,
    .enc_channel = TIM_CHANNEL_2,
};

void Test_Drivetrain_Init(void)
{
    if (drivetrain_init(&wheels, &left_wheel, &right_wheel) != HAL_OK) {
        printf("[Warning] an encoder failed to start; motors still usable\r\n");
    }
    drivetrain_stop(&wheels);

    printf("\r\n========================================\r\n");
    printf("   Drivetrain test - PUT IT ON BLOCKS\r\n");
    printf("   PWM TIM3 CH1/CH2, encoders TIM4 CH1/CH2\r\n");
    printf("   dir pins PB10 (left) and PA4 (right)\r\n");
    printf("========================================\r\n");
}

/* ---------------------------------------------------------------------
 * 1. WHICH WAY DOES EACH WHEEL TURN?
 *
 *    left forward 3 s, stop 2 s, right forward 3 s, stop 2 s, repeat.
 *    Both should turn so the robot WOULD move forwards.
 * ------------------------------------------------------------------ */
void Test_Drivetrain_Wheels_Run(void)
{
    static const char *names[4] = {
        "LEFT wheel forward  - which way does it turn?",
        "stop",
        "RIGHT wheel forward - which way does it turn?",
        "stop",
    };
    static const float    lefts[4]     = { DRIVE_SPEED, 0.0f, 0.0f,          0.0f };
    static const float    rights[4]    = { 0.0f,          0.0f, DRIVE_SPEED, 0.0f };
    static const uint32_t durations[4] = { 3000U,         2000U, 3000U,        2000U };

    static uint32_t phase_start = 0;
    static uint8_t  phase   = 0;
    static uint8_t  started = 0;

    if (!started) {
        started = 1;
        phase = 0;
        phase_start = HAL_GetTick();
        drivetrain_set(&wheels, lefts[0], rights[0]);
        printf("[WHEELS] %s\r\n", names[0]);
    }

    if ((HAL_GetTick() - phase_start) >= durations[phase]) {
        phase = (uint8_t)((phase + 1U) % 4U);
        phase_start = HAL_GetTick();
        drivetrain_set(&wheels, lefts[phase], rights[phase]);
        printf("[WHEELS] %s\r\n", names[phase]);
    }

    test_left_rpm  = drivetrain_rpm(&wheels, LEFT);
    test_right_rpm = drivetrain_rpm(&wheels, RIGHT);
}

/* ---------------------------------------------------------------------
 * 2. CYCLE THROUGH EVERY MOVEMENT
 * ------------------------------------------------------------------ */
typedef struct {
    const char *name;
    float       left;
    float       right;
    uint32_t    ms;
} Move;

static const Move moves[] = {
    { "stop",        0.0f,              0.0f,              1500U },
    { "forward",     DRIVE_SPEED,     DRIVE_SPEED,     2000U },
    { "stop",        0.0f,              0.0f,              1500U },
    { "backward",   -DRIVE_SPEED,    -DRIVE_SPEED,     2000U },
    { "stop",        0.0f,              0.0f,              1500U },
    { "spin left",  -TURN_SPEED,      TURN_SPEED,      1500U },
    { "stop",        0.0f,              0.0f,              1500U },
    { "spin right",  TURN_SPEED,     -TURN_SPEED,      1500U },
    { "stop",        0.0f,              0.0f,              1500U },
    { "arc left",    ARC_SPEED, DRIVE_SPEED,     2000U },
    { "stop",        0.0f,              0.0f,              1500U },
    { "arc right",   DRIVE_SPEED,     ARC_SPEED, 2000U },
    { "stop",        0.0f,              0.0f,              2500U },
};

#define MOVE_COUNT  (sizeof(moves) / sizeof(moves[0]))

void Test_Drivetrain_Run(void)
{
    static uint32_t step_start  = 0;
    static uint32_t last_status = 0;
    static uint8_t  step        = 0;
    static uint8_t  started     = 0;

    if (!started) {
        started = 1;
        step = 0;
        step_start = HAL_GetTick();
        drivetrain_set(&wheels, moves[0].left, moves[0].right);
        printf("[DRIVE] %s  (L %+d%%  R %+d%%)\r\n",
               moves[0].name,
               (int)moves[0].left, (int)moves[0].right);
    }

    if ((HAL_GetTick() - step_start) >= moves[step].ms) {
        step = (uint8_t)((step + 1U) % MOVE_COUNT);
        step_start = HAL_GetTick();
        test_move = step;

        drivetrain_set(&wheels, moves[step].left, moves[step].right);
        printf("[DRIVE] %s  (L %+d%%  R %+d%%)\r\n",
               moves[step].name,
               (int)moves[step].left, (int)moves[step].right);
    }

    test_left_rpm  = drivetrain_rpm(&wheels, LEFT);
    test_right_rpm = drivetrain_rpm(&wheels, RIGHT);

    if ((HAL_GetTick() - last_status) >= 1000U) {
        last_status = HAL_GetTick();
        /* nano printf has no %f, so RPM prints as whole numbers */
        printf("        rpm  L=%4d  R=%4d\r\n",
               (int)test_left_rpm, (int)test_right_rpm);
    }

/* =====================================================================
 * ULTRASONIC SENSOR + MOTOR TEST
 *
 *  Pins:  PB5 -> TRIG
 *         PA8 <- ECHO  (through a voltage divider, 5 V -> 3.3 V)
 *         PA0  -> motor PWM   (TIM2_CH1)
 *         PB10 -> motor direction
 *         PB6  <- encoder FG  (TIM4_CH1)
 *
 *  The Ultrasonic sensor pings every 60 ms. If the distance is less than
 *  20 cm, the motor stops. Otherwise, the motor runs at 75% power in direction 0.
    *  Watch: test_ultrasonic_distance_cm, test_motor_rpm
 * ===================================================================== */

void Test_Ultrasonic_Motor_Init(void)
{
    init_default_motor(&test_motor, &htim2, TIM_CHANNEL_1, GPIOB, GPIO_PIN_10);
    init_encoder(&test_encoder, &htim4, TIM_CHANNEL_1);
    drive_motor(&test_motor, 0.0f, 0U);

    if (HCSR04_Init(&test_ultrasonic, GPIOB, GPIO_PIN_5,
                    GPIOA, GPIO_PIN_8, &htim1) != HAL_OK) {
        Error_Handler();
    }
}

void Test_Ultrasonic_Motor_Run(void)
{
    // 1. Read the ultrasonic sensor
    float distance_cm = -1.0f;
    test_ultrasonic_status = HCSR04_ReadDistance(&test_ultrasonic, &distance_cm);
    test_ultrasonic_distance_cm = distance_cm;
    
    // 2. Drive the motor based on the distance
    if (distance_cm >= 0.0f && distance_cm < 20.0f) {
        // Object detected within 20 cm, stop the motor
        drive_motor(&test_motor, 0.0f, 0U);
    } else {
        // No object detected or distance is greater than 20 cm, run the motor at 75% power in direction 0
        drive_motor(&test_motor, 75.0f, 0U);
    }   

    HAL_Delay(60U);
}