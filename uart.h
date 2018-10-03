/*
 * uart.h
 *
 *  Created on: 2018. szept. 29.
 *      Author: MAJXAAPPTE
 */

#ifndef UART_H_
#define UART_H_


/**
 *  Configuration
 * */

#define UART_BAUD_RATE 9600
#define UART_CLK 12                                                      // SMCLK in MHz
#define BAUD_SETTING_VALUE (UART_CLK*1000000 / UART_BAUD_RATE)

// END configuration


/**
 *  Function prototypes
 * */
void uart_init();

int uart_send_ibutton_data(uint8_t *ib_code);

int uart_send_byte_data(uint8_t data);

static void code_conversion(uint8_t *code, char *buffer);



#endif /* UART_H_ */
