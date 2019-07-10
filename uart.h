/**
 * uart.h
 *
 *  Created on: 2018. szept. 29.
 *      Author: MAJXAAPPTE
 */

#ifndef UART_H_
#define UART_H_


/*________________________________ START SETTINGS ________________________________ */
#define UART_BAUD_RATE 9600
#define UART_CLK 1500000                                                      // SMCLK (Hz)
#define BAUD_SETTING_VALUE (UART_CLK / UART_BAUD_RATE)

#define RX_DATA_SIZE 128
#define UART_TIMEOUT_MS 200

#define TX_BUFFER_SIZE 50

#define START_BYTE 0x55
/*________________________________ END SETTINGS __________________________________ */

/** UART command types */
enum UART_cmd_type{TYPE_ECHO = 1, TYPE_INFO, TYPE_TEST, TYPE_SEND_SETTINGS,
    TYPE_SEND_DATA_CODES, TYPE_SAVE_SETTINGS, TYPE_SAVE_DATA_CODES, TYPE_WRITE_A_KEY, TYPE_OK};

/** UART error codes, defined TYPE_INFO data values. */
#define ERR_TIMEOUT     69
#define ERR_CRC         2

/** UART RX packet */
typedef struct Packet {
    uint8_t type_b;
    uint8_t data_size;
    uint8_t data[RX_DATA_SIZE];
    uint16_t crc;
} packet_t;

void uart_init();
int uart_send_packet(uint8_t *data, uint8_t type, uint8_t data_size);
void uart_send_flash_segment(uint8_t *segm_start_ptr, uint8_t segm_ID);
void uart_timeout();

/** Declare these variables. */
volatile extern uint8_t RX_is_packet;
volatile extern packet_t RX_packet;
volatile extern uint16_t uart_timeot_ms;
volatile extern uint8_t uart_timeout_ticking;

#endif /* UART_H_ */
