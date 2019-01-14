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

#define RX_TX_DATA_SIZE 51

/*________________________________ END SETTINGS ________________________________ */

void uart_init();
void uart_process_command();
int uart_send(uint8_t *data, uint8_t cmd, uint8_t data_size);

volatile extern uint8_t RX_is_packet;


#endif /* UART_H_ */
