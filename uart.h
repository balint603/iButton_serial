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
#define UART_CLK 12                                                      // SMCLK in MHz
#define BAUD_SETTING_VALUE (UART_CLK*1000000 / UART_BAUD_RATE)
/*________________________________ END SETTINGS ________________________________ */

void uart_init();

int uart_send_ibutton_data(uint8_t *ib_code, uint8_t send_as_string);

int uart_send_byte(uint8_t byte);

int uart_send_str(char *str, uint8_t new_line);

uint8_t uart_get_byte();

void hex_byte_to_char(uint8_t code, char *MSB, char *LSB);

volatile extern uint8_t uart_rx_buffer_not_empty_flag;
volatile extern uint8_t uart_rx_buffer_full;
volatile extern uint8_t uart_rx_buffer_ovf_flag;
volatile extern uint8_t uart_tx_buffer_not_empty_flag;
volatile extern uint8_t uart_tx_buffer_full;
volatile extern uint8_t uart_tx_buffer_ovf_flag;

#endif /* UART_H_ */
