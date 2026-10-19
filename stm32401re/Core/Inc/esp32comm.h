/*

    author: nicholai

    date: oct 19, 2026

    description: header file for ESP32 <-> STM32 UART packet driver

    Packet format (shared with the ESP32 sketches in uart_commtest/):

        0xAA 0x55 <type> <len> <payload: len bytes> <crc8>

        0xAA 0x55  sync bytes, mark the start of a packet
        type       what the packet means (see ESP32_PacketType)
        len        number of payload bytes, 0..ESP32_MAX_PAYLOAD
        payload    the data
        crc8       crc over type, len and payload (poly 0x31, init 0x00)

    Usage:
        ESP32_Comm_Init(&huart1);                  // once, after MX_USART1_UART_Init()
        while (ESP32_Comm_GetPacket(&pkt)) { ... } // in the main loop
*/

#pragma once

#include "stm32f4xx_hal.h"

#define ESP32_SYNC1        0xAAU
#define ESP32_SYNC2        0x55U
#define ESP32_MAX_PAYLOAD  8U

typedef enum {
    ESP32_TYPE_PING  = 0x01,  // payload: seq (uint8), echoed back by the STM32
    ESP32_TYPE_MOTOR = 0x02,  // payload: speed (int8, -100..100, sign = direction)
} ESP32_PacketType;

typedef struct {
    uint8_t type;                        // ESP32_PacketType
    uint8_t len;                         // number of valid bytes in payload
    uint8_t payload[ESP32_MAX_PAYLOAD];
    uint8_t crc_ok;                      // 1 if crc matched, 0 if corrupted
} ESP32_Packet;

typedef struct {
    uint32_t good;     // packets with a valid crc
    uint32_t bad_crc;  // packets with a wrong crc
    uint32_t junk;     // stray bytes / packets with an impossible len
} ESP32_Stats;

HAL_StatusTypeDef ESP32_Comm_Init(UART_HandleTypeDef *huart);
uint8_t ESP32_Comm_GetPacket(ESP32_Packet *pkt);
HAL_StatusTypeDef ESP32_Comm_SendPacket(uint8_t type, const uint8_t *payload, uint8_t len);
const ESP32_Stats *ESP32_Comm_GetStats(void);
