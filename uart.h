/*
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

#define RX_DATA_SIZE 102
#define UART_TIMEOUT_MS 200


/*________________________________ END SETTINGS ________________________________ */

/** UART command types */
enum UART_cmd_type{CMD_ECHO = 1, CMD_INFO};

void uart_init();
void uart_process_command();
int uart_send(uint8_t *data, uint8_t cmd, uint8_t data_size);
void uart_timeout();

volatile extern uint8_t RX_is_packet;
volatile extern uint16_t uart_timeot_ms;
volatile extern uint8_t uart_timeout_ticking;

#endif /* UART_H_ */
