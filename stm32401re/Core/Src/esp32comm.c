/*
 * esp32comm.c
 *
 *  Created on: Oct 19, 2026
 *      Author: nicholai
 *
 *  Interrupt-driven UART receive + packet parser for the ESP32 link.
 *  The UART itself (pins, baud, NVIC) is configured in CubeMX.
 *  Packet format is described in esp32comm.h.
 */

#include "esp32comm.h"

#define ESP32_TX_TIMEOUT_MS 10U

/* type + len + payload, the bytes the crc covers */
#define ESP32_CRC_SPAN_MAX  (2U + ESP32_MAX_PAYLOAD)

static UART_HandleTypeDef *esp_uart = NULL;
static ESP32_Stats stats;

/* Bytes from the ESP32 land here from the UART interrupt, so nothing
 * is dropped while the main loop is busy (e.g. printf).
 * uint8_t indexes wrap at 256 for free. */
static uint8_t rx_byte;
static volatile uint8_t rx_buf[256];
static volatile uint8_t rx_head = 0;  // written by interrupt
static volatile uint8_t rx_tail = 0;  // read by ESP32_Comm_GetPacket

static uint8_t ESP32_Crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

HAL_StatusTypeDef ESP32_Comm_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return HAL_ERROR;
    }

    esp_uart = huart;
    stats = (ESP32_Stats){0};
    rx_head = 0;
    rx_tail = 0;

    // Start listening for the first byte; the callback keeps it going
    return HAL_UART_Receive_IT(esp_uart, &rx_byte, 1);
}

/* Returns 1 and fills *pkt when a full packet has been received, 0 if
 * no complete packet is waiting. Call it in a loop until it returns 0.
 * Always check pkt->crc_ok before using the payload. */
uint8_t ESP32_Comm_GetPacket(ESP32_Packet *pkt)
{
    enum { WAIT_SYNC1, WAIT_SYNC2, READ_TYPE, READ_LEN, READ_PAYLOAD, READ_CRC };

    // Parser state, kept between calls
    static uint8_t state = WAIT_SYNC1;
    static uint8_t frame[ESP32_CRC_SPAN_MAX];  // [type, len, payload...]
    static uint8_t payload_count = 0;

    while (rx_tail != rx_head) {
        uint8_t byte = rx_buf[rx_tail];
        rx_tail++;

        switch (state) {
        case WAIT_SYNC1:
            if (byte == ESP32_SYNC1) {
                state = WAIT_SYNC2;
            } else {
                stats.junk++;
            }
            break;

        case WAIT_SYNC2:
            if (byte == ESP32_SYNC2) {
                state = READ_TYPE;
            } else if (byte != ESP32_SYNC1) {  // another 0xAA may be a new start
                stats.junk++;
                state = WAIT_SYNC1;
            }
            break;

        case READ_TYPE:
            frame[0] = byte;
            state = READ_LEN;
            break;

        case READ_LEN:
            if (byte > ESP32_MAX_PAYLOAD) {  // can't be a real packet, resync
                stats.junk++;
                state = WAIT_SYNC1;
                break;
            }
            frame[1] = byte;
            payload_count = 0;
            state = (byte == 0U) ? READ_CRC : READ_PAYLOAD;
            break;

        case READ_PAYLOAD:
            frame[2U + payload_count] = byte;
            payload_count++;
            if (payload_count >= frame[1]) {
                state = READ_CRC;
            }
            break;

        case READ_CRC:
            state = WAIT_SYNC1;
            pkt->type = frame[0];
            pkt->len = frame[1];
            for (uint8_t i = 0; i < pkt->len; i++) {
                pkt->payload[i] = frame[2U + i];
            }
            pkt->crc_ok = (byte == ESP32_Crc8(frame, (uint8_t)(2U + pkt->len)));

            if (pkt->crc_ok) {
                stats.good++;
            } else {
                stats.bad_crc++;
            }
            return 1;
        }
    }

    return 0;
}

/* Sends 0xAA 0x55 <type> <len> <payload> <crc8> to the ESP32 */
HAL_StatusTypeDef ESP32_Comm_SendPacket(uint8_t type, const uint8_t *payload, uint8_t len)
{
    if (esp_uart == NULL || len > ESP32_MAX_PAYLOAD || (len > 0U && payload == NULL)) {
        return HAL_ERROR;
    }

    uint8_t pkt[2U + ESP32_CRC_SPAN_MAX + 1U];  // sync + type/len/payload + crc
    pkt[0] = ESP32_SYNC1;
    pkt[1] = ESP32_SYNC2;
    pkt[2] = type;
    pkt[3] = len;
    for (uint8_t i = 0; i < len; i++) {
        pkt[4U + i] = payload[i];
    }
    pkt[4U + len] = ESP32_Crc8(&pkt[2], (uint8_t)(2U + len));

    return HAL_UART_Transmit(esp_uart, pkt, (uint16_t)(5U + len), ESP32_TX_TIMEOUT_MS);
}

const ESP32_Stats *ESP32_Comm_GetStats(void)
{
    return &stats;
}

/* NOTE: these HAL callbacks are shared by every UART and can only be
 * defined once in the project. If another UART needs receive callbacks,
 * add its handling here instead of defining them again elsewhere. */

/* HAL calls this after each received byte: store it, then listen again */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != esp_uart) {
        return;
    }

    uint8_t next = (uint8_t)(rx_head + 1U);
    if (next != rx_tail) {  // drop byte if buffer is full
        rx_buf[rx_head] = rx_byte;
        rx_head = next;
    }
    HAL_UART_Receive_IT(esp_uart, &rx_byte, 1);
}

/* HAL stops receiving on an error (e.g. overrun/noise): restart it */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == esp_uart) {
        HAL_UART_Receive_IT(esp_uart, &rx_byte, 1);
    }
}
