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


volatile extern uint8_t RX_is_packet;
volatile extern uint8_t TX_is_packet;


#endif /* UART_H_ */
